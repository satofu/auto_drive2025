#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  // initialize parameters
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  low_speed_threshold_(declare_parameter<float>("low_speed_threshold", 1.5)),
  lookahead_gain_low_speed_(declare_parameter<float>("lookahead_gain_low_speed", 0.1)),
  lookahead_min_distance_low_speed_(declare_parameter<float>("lookahead_min_distance_low_speed", 1.2)),
  speed_proportional_gain_low_speed_(declare_parameter<float>("speed_proportional_gain_low_speed", 0.8)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  steering_tire_angle_gain_(declare_parameter<float>("steering_tire_angle_gain", 1.0)),
  gnss_to_front_axle_offset_(
    declare_parameter<float>("gnss_to_front_axle_offset", 0.0))
{
  start_time_ = get_clock()->now();
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);

  const auto bv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", bv_qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 10ms, std::bind(&SimplePurePursuit::onTimer, this));
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

void SimplePurePursuit::onTimer()
{
  rclcpp::Time now = get_clock()->now();
  if ((now - start_time_) < rclcpp::Duration::from_seconds(5.0)) {
    auto cmd = zeroAckermannControlCommand(now);
    pub_cmd_->publish(cmd);
    pub_raw_cmd_->publish(cmd);
    return;
  }

  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  double yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  geometry_msgs::msg::Point front_point;
  front_point.x =
    odometry_->pose.pose.position.x + gnss_to_front_axle_offset_ * std::cos(yaw);
  front_point.y =
    odometry_->pose.pose.position.y + gnss_to_front_axle_offset_ * std::sin(yaw);
  front_point.z = odometry_->pose.pose.position.z;

  size_t closet_traj_point_idx =
    findNearestIndex(trajectory_->points, front_point);

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(now);

  // get closest trajectory point from current position
  TrajectoryPoint closet_traj_point = trajectory_->points.at(closet_traj_point_idx);

  // calc longitudinal speed and acceleration
  double current_longitudinal_vel = odometry_->twist.twist.linear.x;
  double target_longitudinal_vel =
    use_external_target_vel_ ? external_target_vel_ : closet_traj_point.longitudinal_velocity_mps;
  double lookahead_gain = lookahead_gain_;
  double lookahead_min_distance = lookahead_min_distance_;
  double speed_proportional_gain = speed_proportional_gain_;

  if (current_longitudinal_vel < low_speed_threshold_) {
    target_longitudinal_vel = low_speed_threshold_ * 1.05;
    lookahead_gain = lookahead_gain_low_speed_;
    lookahead_min_distance = lookahead_min_distance_low_speed_;
    speed_proportional_gain = speed_proportional_gain_low_speed_;
    double desired_acc =
      speed_proportional_gain * (target_longitudinal_vel - current_longitudinal_vel);
    cmd.longitudinal.acceleration = std::min(desired_acc, 1.0);
  } else {
    cmd.longitudinal.acceleration =
      speed_proportional_gain * (target_longitudinal_vel - current_longitudinal_vel);
  }

  cmd.longitudinal.speed = target_longitudinal_vel;

  // calc lateral control
  //// calc lookahead distance
  double lookahead_distance = lookahead_gain * target_longitudinal_vel + lookahead_min_distance;
  //// calc center coordinate of rear wheel
  double rear_x = front_point.x - wheel_base_ * std::cos(yaw);
  double rear_y = front_point.y - wheel_base_ * std::sin(yaw);
  //// search lookahead point
  auto lookahead_point_itr = std::find_if(
    trajectory_->points.begin() + closet_traj_point_idx, trajectory_->points.end(),
    [&](const TrajectoryPoint & point) {
      return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >=
             lookahead_distance;
    });
  double lookahead_point_x = lookahead_point_itr->pose.position.x;
  double lookahead_point_y = lookahead_point_itr->pose.position.y;

  geometry_msgs::msg::PointStamped lookahead_point_msg;
  lookahead_point_msg.header.stamp = get_clock()->now();
  lookahead_point_msg.header.frame_id = "map";
  lookahead_point_msg.point.x = lookahead_point_x;
  lookahead_point_msg.point.y = lookahead_point_y;
  lookahead_point_msg.point.z = closet_traj_point.pose.position.z;
  pub_lookahead_point_->publish(lookahead_point_msg);

  // calc steering angle for lateral control
  double alpha =
    std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) - yaw;
  // if target point is behind the vehicle, switch to reverse and flip steering
  bool target_is_behind = std::abs(alpha) > (M_PI / 2.0);
  if (target_is_behind) {
    cmd.longitudinal.speed *= -1.0;
    cmd.longitudinal.acceleration *= -1.0;
  }
  double steer = steering_tire_angle_gain_ *
                 std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);
  if (target_is_behind) {
    steer *= -1.0;
  }
  cmd.lateral.steering_tire_angle = steer;

  pub_cmd_->publish(cmd);
  cmd.lateral.steering_tire_angle /=  steering_tire_angle_gain_;
  pub_raw_cmd_->publish(cmd);
}

bool SimplePurePursuit::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
    return false;
  }
  if (trajectory_->points.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/,  "trajectory points is empty");
      return false;
    }
  return true;
}
}  // namespace simple_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
