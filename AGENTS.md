# Task: Launch一括管理での最小改造（低速で完走するための基礎設定）

## 目的

`simple_pure_pursuit` のパラメータを **1つの Launch**（`reference.launch.xml`）内で最小変更し、**一定速＋Pure Pursuit**でまず安定完走させる。

## 対象リポジトリ

* このリポジトリ（`auto_drive2025`）
* 参照ファイル: `aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/reference.launch.xml`
  ※上記パスに無い場合はリポ内検索で解決する（下記手順1に検索コマンドあり）。

---

## 実装手順

### 1) ブランチ作成 & 参照ファイルの所在確認

```
cd ~/auto_drive2025
git switch -c feat/launch-minimal-pp

# reference.launch.xml の所在と PP ノードの所在を検索
grep -R --line-number "reference.launch.xml" aichallenge || true
grep -R --line-number "simple_pure_pursuit" aichallenge/workspace/src || true
```

### 2) `reference.launch.xml` のバックアップ

```
LAUNCH="aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/reference.launch.xml"
cp -v "$LAUNCH" "./output/reference.launch.xml.bak.$(date +%Y%m%d-%H%M%S)"
```

### 3) `simple_pure_pursuit` パラメータ編集（最低限）

* `simple_pure_pursuit` ノードブロック内を**以下の値に設定**する。

  * `use_external_target_vel="true"`
  * `external_target_vel="8.0"`（挙動が不安定なら 6.0–7.0 に下げて再評価）
  * `lookahead_gain="0.4"`
  * `lookahead_min_distance="5.0"`
  * `speed_proportional_gain="1.0"`

**例（該当ノード内に追加/上書き）:**

```xml
<node pkg="simple_pure_pursuit" exec="simple_pure_pursuit" name="simple_pure_pursuit_node" output="screen">
  <param name="use_external_target_vel" value="true"/>
  <param name="external_target_vel" value="8.0"/>
  <param name="lookahead_gain" value="0.4"/>
  <param name="lookahead_min_distance" value="5.0"/>
  <param name="speed_proportional_gain" value="1.0"/>
</node>
```

### 4) ビルド & 実行（評価）

```
# リポ直下で
./aichallenge/build_autoware.bash

# シミュレーション実行（描画なしAWSIM + Autoware）
./aichallenge/run_evaluation.bash
```

**期待挙動**: 蛇行せず周回できること。コーナで外に膨らむ等があれば次ステップで微調整。

### 5) 最小チューニングの指針

* 直進でフラつく → `lookahead_min_distance` を **+0.5〜+1.0**
* コーナ外側に膨らむ → `lookahead_min_distance` を **-0.5** するか `external_target_vel` を **-1.0 m/s**
* 進入オーバー傾向 → まずは **速度を下げる**（速度計画は次フェーズで対応）

※ パラメータ変更のたびに **再ビルド→再実行**。

### 6) 変更をコミット & プッシュ

```
git add "$LAUNCH"
git commit -m "chore(launch): minimal Pure Pursuit baseline (ext vel, lookahead, speed gain)"
git push -u origin feat/launch-minimal-pp
```

### 7) ロールバック

```
git restore --source=HEAD~1 -- "$LAUNCH"  # 直前コミットを戻す例
# またはバックアップから復元
cp -v "./output/reference.launch.xml.bak.YYYYMMDD-HHMMSS" "$LAUNCH"
```

---

## 参考URL（平文）

* セットアップ要件
  [https://automotiveaichallenge.github.io/aichallenge-documentation-2025/setup/requirements.html](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/setup/requirements.html)
* 開発ワークスペースの使い方
  [https://automotiveaichallenge.github.io/aichallenge-documentation-2025/development/workspace-usage.html](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/development/workspace-usage.html)
* メインモジュール（起動や構成の概要）
  [https://automotiveaichallenge.github.io/aichallenge-documentation-2025/development/main-module.html](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/development/main-module.html)
* 仕様：インターフェース（トピック/メッセージ型の確認）
  [https://automotiveaichallenge.github.io/aichallenge-documentation-2025/specifications/interface.html](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/specifications/interface.html)
* 仕様：シミュレータ（AWSIM関連）
  [https://automotiveaichallenge.github.io/aichallenge-documentation-2025/specifications/simulator.html](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/specifications/simulator.html)
* コース（コース/車両/速度計画/回避/自己位置推定）
  [https://automotiveaichallenge.github.io/aichallenge-documentation-2025/course/index.html](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/course/index.html)
  [https://automotiveaichallenge.github.io/aichallenge-documentation-2025/course/vehicle.html](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/course/vehicle.html)
  [https://automotiveaichallenge.github.io/aichallenge-documentation-2025/course/velocity_planning.html](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/course/velocity_planning.html)
  [https://automotiveaichallenge.github.io/aichallenge-documentation-2025/course/avoidance.html](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/course/avoidance.html)
  [https://automotiveaichallenge.github.io/aichallenge-documentation-2025/course/localization.html](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/course/localization.html)
* FAQ
  [https://automotiveaichallenge.github.io/aichallenge-documentation-2025/faq.html](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/faq.html)
* AI（実装の進め方の全体像）
  [https://automotiveaichallenge.github.io/aichallenge-documentation-2025/ai.html](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/ai.html)

---
