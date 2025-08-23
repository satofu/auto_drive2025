#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GA optimizer for raceline_awsim_30km_from_garage.csv

- Head 10 rows are fixed (not optimized).
- Genes = per-row offsets [dx, dy, dz, dv] for rows [10..end].
- Constraints:
  - dx,dy,dz in [-1.0, +1.0], quantized to 0.1
  - speed max (base_v + dv) <= 9.72 m/s, dv quantized to 0.1 (you can change in settings)
- Fitness:
  - Run external emulator script which generates simulation_output.csv
  - Fitness = total XY path length (longer is better)
- Artifacts:
  - ga/ga_settings.json (auto-created)
  - ga/data/ (all simulation_output.csv copies)
  - ga/gen_XXX_summary.csv (each generation summary)
  - ga/best_offsets.csv, ga/best_solution.csv (final outputs)

Run in WSL. Requires: pandas, numpy, tqdm.
"""

import os
import sys
import json
import time
import math
import shutil
import subprocess
from pathlib import Path
from datetime import datetime

import numpy as np
import pandas as pd

try:
    from tqdm import tqdm
except Exception:
    tqdm = lambda x, **k: x  # fallback: no progress bar


# ---------- constants & default settings ----------
REPO_ROOT = os.path.expanduser('~/aichallenge-2025')
DATA_DIR = os.path.join(REPO_ROOT, 'aichallenge/workspace/src/aichallenge_submit/simple_trajectory_generator/data')
GA_DIR = os.path.join(DATA_DIR, 'ga')
GA_DATA_DIR = os.path.join(GA_DIR, 'data')  # bulk csv storage
SETTINGS_PATH = os.path.join(GA_DIR, 'ga_settings.json')

WORKING_RACELINE = os.path.join(DATA_DIR, 'raceline_awsim_30km_from_garage.csv')  # overwritten each eval
EMULATOR_SCRIPT = os.path.join(DATA_DIR, 'awsim_emulator.py')  # executed with cwd=REPO_ROOT
EMULATOR_OUTPUT = os.path.join(DATA_DIR, 'simulation_output.csv')  # produced by emulator

DEFAULT_SETTINGS = {
    "fixed_head_rows": 10,         # head rows are fixed
    "generations": 5,
    "population_size": 12,
    "elite_fraction": 0.20,        # top x kept as-is to next gen
    "parent_fraction": 0.50,       # top x used as parents for crossover
    "mutation_rate": 0.20,         # per-gene mutation probability
    "xyz_step": 0.1,
    "xyz_bound": 1.0,
    "speed_step": 0.1,
    "speed_max": 9.72,
    "speed_min": 0.0,
    "emulator_timeout_sec": 300,   # 5 min per eval (必要に応じ変更)
    "random_seed": 42
}


# ---------- utilities ----------
def ensure_dirs():
    os.makedirs(GA_DIR, exist_ok=True)
    os.makedirs(GA_DATA_DIR, exist_ok=True)


def load_or_create_settings():
    ensure_dirs()
    if not os.path.exists(SETTINGS_PATH):
        with open(SETTINGS_PATH, 'w', encoding='utf-8') as f:
            json.dump(DEFAULT_SETTINGS, f, indent=2, ensure_ascii=False)
        print(f"[INFO] settings created: {SETTINGS_PATH}")
        return DEFAULT_SETTINGS.copy()
    with open(SETTINGS_PATH, 'r', encoding='utf-8') as f:
        st = json.load(f)
    # fill missing defaults (forward compatibility)
    for k, v in DEFAULT_SETTINGS.items():
        st.setdefault(k, v)
    return st


def quantize(value, step):
    return round(value / step) * step


def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))

def read_csv_loose(path):
    """
    raceline CSV を読み込む。
    期待カラム: x,y,z,v（先頭4列を使用）。文字列で入っていても数値に強制変換。
    数値化できない行（ヘッダ行等）は除外する。
    """
    # コメント行(#)を無視し、ヘッダなしで読み込む
    df = pd.read_csv(path, header=None, comment='#')

    if df.shape[1] < 4:
        raise ValueError(f"CSV {path} must have at least 4 columns (x,y,z,v). Found {df.shape[1]}.")

    # 先頭4列のみ使う
    df = df.iloc[:, :4].copy()

    # 数値に強制変換（できなければ NaN）
    for c in df.columns:
        df[c] = pd.to_numeric(df[c], errors='coerce')

    # 数値化できなかった行（ヘッダ行など）を落とす
    before = len(df)
    df = df.dropna().reset_index(drop=True)
    removed = before - len(df)
    if removed > 0:
        print(f"[WARN] 非数値行を {removed} 行スキップしました（ヘッダ等）。")

    df.columns = ['x', 'y', 'z', 'v']
    return df


def write_raceline_csv(df, path):
    # v 列を speed にリネームして、ヘッダ付きで書き出す
    out = df.rename(columns={'v': 'speed'})
    out[['x', 'y', 'z', 'speed']].to_csv(path, header=True, index=False, float_format='%.6f')


def compute_xy_path_length(csv_path):
    """
    Load simulation_output.csv and compute total 2D path length over (x,y).
    Column guessing: try header names first; fallback to first two columns.
    """
    df = pd.read_csv(csv_path)
    # guess columns
    xcol, ycol = None, None
    for cand in ['x', 'X', 'pos_x', 'px', 'X[m]']:
        if cand in df.columns:
            xcol = cand; break
    for cand in ['y', 'Y', 'pos_y', 'py', 'Y[m]']:
        if cand in df.columns:
            ycol = cand; break
    if xcol is None or ycol is None:
        # fallback: first two numeric columns
        num_cols = [c for c in df.columns if np.issubdtype(df[c].dtype, np.number)]
        if len(num_cols) >= 2:
            xcol, ycol = num_cols[0], num_cols[1]
        else:
            raise ValueError("simulation_output.csv に x,y の数値列が見つかりません。")

    x = df[xcol].to_numpy(dtype=float)
    y = df[ycol].to_numpy(dtype=float)
    if len(x) < 2:
        return 0.0
    dx = np.diff(x)
    dy = np.diff(y)
    dist = np.sqrt(dx*dx + dy*dy).sum()
    return float(dist)


def run_emulator(timeout_sec):
    """
    Run external emulator script that reads WORKING_RACELINE and writes EMULATOR_OUTPUT.
    """
    if os.path.exists(EMULATOR_OUTPUT):
        try:
            os.remove(EMULATOR_OUTPUT)
        except Exception:
            pass

    # Run: python3 aichallenge/.../awsim_emulator.py  with cwd=REPO_ROOT
    cmd = [sys.executable, EMULATOR_SCRIPT]  # use current python3
    try:
        res = subprocess.run(
            cmd, cwd=REPO_ROOT,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=timeout_sec, text=True
        )
    except subprocess.TimeoutExpired:
        return False, f"emulator timeout (> {timeout_sec}s)"
    except Exception as e:
        return False, f"emulator failed: {e}"

    if res.returncode != 0:
        return False, f"emulator returncode={res.returncode}\nSTDERR:\n{res.stderr}\nSTDOUT:\n{res.stdout}"

    if not os.path.exists(EMULATOR_OUTPUT):
        return False, "emulator finished but simulation_output.csv not found"

    return True, res.stdout.strip()


# ---------- GA core ----------
class GA:
    def __init__(self, base_df, settings):
        self.base_df = base_df.copy()
        self.st = settings
        self.head = int(self.st["fixed_head_rows"])

        # rows to optimize
        if len(self.base_df) <= self.head:
            raise ValueError(f"Base CSV has only {len(self.base_df)} rows; needs > {self.head}.")

        self.n_genes = len(self.base_df) - self.head   # rows after head
        self.gene_dim = 4  # dx,dy,dz,dv
        self.pop_size = int(self.st["population_size"])
        self.elite_k = max(1, int(self.pop_size * float(self.st["elite_fraction"])))
        self.parent_k = max(self.elite_k, int(self.pop_size * float(self.st["parent_fraction"])))
        self.mut_rate = float(self.st["mutation_rate"])

        self.xyz_step = float(self.st["xyz_step"])
        self.xyz_bound = float(self.st["xyz_bound"])
        self.speed_step = float(self.st["speed_step"])
        self.speed_max = float(self.st["speed_max"])
        self.speed_min = float(self.st["speed_min"])

        np.random.seed(int(self.st["random_seed"]))

    def init_population(self):
        # start near zero offsets
        # random uniform in [-0.5, 0.5], then quantize
        rand = np.random.uniform(-0.5, 0.5, size=(self.pop_size, self.n_genes, self.gene_dim))
        # quantize xyz/dv separately
        for i in range(self.pop_size):
            # xyz
            for j in range(self.n_genes):
                for k in range(3):
                    rand[i, j, k] = quantize(rand[i, j, k], self.xyz_step)
                    rand[i, j, k] = clamp(rand[i, j, k], -self.xyz_bound, self.xyz_bound)
                # dv
                rand[i, j, 3] = quantize(rand[i, j, 3], self.speed_step)
        return rand

    def apply_offsets(self, offsets):
        """
        Apply offsets to base_df -> new df (respecting constraints).
        offsets shape: (n_genes, 4)
        """
        df = self.base_df.copy()
        # slice rows for genes
        rows = range(self.head, len(df))
        # x,y,z
        df.loc[rows, 'x'] = df.loc[rows, 'x'].to_numpy() + offsets[:, 0]
        df.loc[rows, 'y'] = df.loc[rows, 'y'].to_numpy() + offsets[:, 1]
        df.loc[rows, 'z'] = df.loc[rows, 'z'].to_numpy() + offsets[:, 2]

        # quantize and clamp xyz displacement relative to base
        for axis in ['x', 'y', 'z']:
            base_vals = self.base_df.loc[rows, axis].to_numpy()
            new_vals = df.loc[rows, axis].to_numpy()
            disp = new_vals - base_vals
            disp = np.clip(quantize_array(disp, self.xyz_step), -self.xyz_bound, self.xyz_bound)
            df.loc[rows, axis] = base_vals + disp

        # speed
        base_v = self.base_df.loc[rows, 'v'].to_numpy()
        dv = offsets[:, 3]
        v_new = base_v + dv
        v_new = quantize_array(v_new, self.speed_step)
        v_new = np.clip(v_new, self.speed_min, self.speed_max)
        df.loc[rows, 'v'] = v_new
        return df

    def evaluate_individual(self, gen_idx, ind_idx, offsets):
        # 1) apply offsets and write working raceline
        df_new = self.apply_offsets(offsets)
        write_raceline_csv(df_new, WORKING_RACELINE)

        # 2) run emulator
        ok, msg = run_emulator(self.st["emulator_timeout_sec"])
        score = -1.0
        sim_copy_path = None
        if ok:
            # 3) compute score
            try:
                score = compute_xy_path_length(EMULATOR_OUTPUT)
            except Exception as e:
                msg = f"compute length error: {e}"
            # 4) copy sim output to GA_DATA_DIR
            ts = datetime.now().strftime('%Y%m%d-%H%M%S')
            sim_copy_path = os.path.join(GA_DATA_DIR, f'gen{gen_idx:03d}_ind{ind_idx:03d}_score{score:.2f}_{ts}.csv')
            try:
                shutil.copy2(EMULATOR_OUTPUT, sim_copy_path)
            except Exception as e:
                msg += f" | copy error: {e}"
        return float(score), ok, msg, sim_copy_path

    def select_parents(self, pop, fitness):
        # rank by fitness desc
        idx = np.argsort(-fitness)
        elites = pop[idx[:self.elite_k]]
        parents = pop[idx[:self.parent_k]]
        return elites, parents, idx

    def crossover(self, parents):
        # produce children to fill population_size
        children_needed = self.pop_size - len(parents)
        children = []
        for _ in range(children_needed):
            pa, pb = parents[np.random.randint(len(parents), size=2)]
            # uniform crossover per gene&dim
            mask = np.random.rand(self.n_genes, self.gene_dim) < 0.5
            child = np.where(mask, pa, pb)
            children.append(child)
        if children:
            children = np.stack(children, axis=0)
            return np.concatenate([parents, children], axis=0)
        else:
            return parents.copy()

    def mutate(self, pop):
        # per-gene mutation: add {-1.0 .. +1.0} * step with prob
        # xyz in 0.1 step, dv in 0.1 step
        for i in range(len(pop)):
            mask = np.random.rand(self.n_genes, self.gene_dim) < self.mut_rate
            # xyz
            xyz_delta = (np.random.randint(-10, 11, size=(self.n_genes, 3)) * self.xyz_step)
            pop[i][:, :3] = np.where(mask[:, :3], pop[i][:, :3] + xyz_delta, pop[i][:, :3])
            # clamp xyz offset
            pop[i][:, :3] = np.clip(pop[i][:, :3], -self.xyz_bound, self.xyz_bound)
            # dv
            dv_delta = (np.random.randint(-10, 11, size=(self.n_genes, 1)) * self.speed_step)
            pop[i][:, 3:4] = np.where(mask[:, 3:4], pop[i][:, 3:4] + dv_delta, pop[i][:, 3:4])
        return pop

    def save_generation_summary(self, gen_idx, rows):
        """
        rows: list of dicts with keys: ind, score, ok, msg, sim_csv
        """
        out_path = os.path.join(GA_DIR, f'gen_{gen_idx:03d}_summary.csv')
        pd.DataFrame(rows).sort_values('score', ascending=False).to_csv(out_path, index=False)

    def save_best(self, gen_idx, best_offsets):
        # save offsets as CSV (dx,dy,dz,dv per row after head)
        off_path = os.path.join(GA_DIR, 'best_offsets.csv')
        off_gen_path = os.path.join(GA_DIR, f'best_offsets_gen{gen_idx:03d}.csv')
        cols = ['dx', 'dy', 'dz', 'dv']
        df_off = pd.DataFrame(best_offsets, columns=cols)
        df_off.to_csv(off_path, index=False)
        df_off.to_csv(off_gen_path, index=False)

        # save final solution CSV (base + best_offsets applied)
        best_df = self.apply_offsets(best_offsets)
        sol_path = os.path.join(GA_DIR, 'best_solution.csv')
        sol_gen_path = os.path.join(GA_DIR, f'best_solution_gen{gen_idx:03d}.csv')
        write_raceline_csv(best_df, sol_path)
        write_raceline_csv(best_df, sol_gen_path)


def quantize_array(arr, step):
    return np.round(arr / step) * step


# ---------- main ----------
def main():
    settings = load_or_create_settings()
    ensure_dirs()

    # ask base CSV path (空なら WORKING_RACELINE の現行を基準にする)
    base_path = input(
        f"基準となる CSV のパスを入力してください。\n"
        f"(空Enterで既定: {WORKING_RACELINE})\n> "
    ).strip()
    if base_path == "":
        base_path = WORKING_RACELINE

    if not os.path.exists(base_path):
        print(f"[ERROR] 基準CSVが見つかりません: {base_path}")
        sys.exit(1)

    # load base CSV
    base_df = read_csv_loose(base_path)

    # initialize GA
    ga = GA(base_df, settings)
    population = ga.init_population()

    best_score_global = -1.0
    best_offsets_global = None

    generations = int(settings["generations"])

    print(f"[INFO] genes(rows)={ga.n_genes}, population={ga.pop_size}, generations={generations}")

    gen_bar = tqdm(range(generations), desc="Generations", unit="gen")
    for gen_idx in gen_bar:
        # evaluate population
        scores = np.zeros((ga.pop_size,), dtype=float)
        info_rows = []
        ind_bar = tqdm(range(ga.pop_size), desc=f"Gen {gen_idx:03d}", unit="ind", leave=False)
        for ind_idx in ind_bar:
            offsets = population[ind_idx]
            score, ok, msg, sim_path = ga.evaluate_individual(gen_idx, ind_idx, offsets)
            scores[ind_idx] = score
            info_rows.append({
                "generation": gen_idx,
                "ind": ind_idx,
                "score": score,
                "ok": ok,
                "sim_csv": sim_path if sim_path else "",
                "note": msg.replace('\n', ' ') if isinstance(msg, str) else ""
            })
            ind_bar.set_postfix_str(f"best={scores.max():.1f}")

        # save summary
        ga.save_generation_summary(gen_idx, info_rows)

        # update global best
        best_idx = int(np.argmax(scores))
        if scores[best_idx] > best_score_global:
            best_score_global = float(scores[best_idx])
            best_offsets_global = population[best_idx].copy()
            ga.save_best(gen_idx, best_offsets_global)

        gen_bar.set_postfix_str(f"best_global={best_score_global:.1f}")

        # selection & next generation
        elites, parents, rank_idx = ga.select_parents(population, scores)
        next_pop = ga.crossover(parents)
        next_pop = ga.mutate(next_pop)

        # preserve elites (replace worst slots)
        # place elites at the front, then fill the rest with next_pop minus its best to avoid duplicates
        # simple approach: sort next_pop by nothing; just overwrite worst idx
        worst_idx = np.argsort(scores)[:len(elites)]
        next_pop[worst_idx] = elites

        population = next_pop

    # final outputs already saved as best_offsets.csv & best_solution.csv
    print(f"[DONE] Best score = {best_score_global:.3f}")
    print(f"[DONE] Outputs:")
    print(f"  - {os.path.join(GA_DIR, 'best_offsets.csv')}")
    print(f"  - {os.path.join(GA_DIR, 'best_solution.csv')}")


if __name__ == "__main__":
    main()

