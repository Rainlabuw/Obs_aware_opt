"""Headline study: multi-Ego satellite proximity operations.

Runs, for a fleet of Ego spacecraft tracking an uncooperative Target:

  1. the three planner variants (estimation-agnostic / own-set / network),
  2. an ablation over fleet size ``m = 1..4``,
  3. the convergence history of Algorithm 1.

Everything is written to ``results/satellite*.{json,npz}``.  No plotting here --
see ``figures.py``.
"""

from __future__ import annotations

import argparse

import numpy as np

import scenario_satellite as S
from common import evaluate, print_table, save, summarise

from maobs import plan  # noqa: E402


def run_case(m: int, label: str, kw: dict, mc_trials: int, verbose: bool = True):
    agents, oracles, x0, x_des = S.make(m)
    cfg = S.config(**kw)
    if verbose:
        print(f"\n--- m={m}  {label}  (alpha_obs={cfg.alpha_obs}, lambda_N={cfg.lambda_net}) ---")
    res = plan(agents, oracles, x0, x_des, S.TARGET, cfg)
    if verbose:
        print(f"    {res.outer_iterations} outer iters, converged={res.converged} "
              f"({res.convergence_reason}), {res.solve_time:.0f} s, "
              f"{res.subproblems_solved} subproblems, {res.inaccurate_solves} inaccurate")
    ev = evaluate(agents, oracles, res.x, S.TARGET, epsilon=S.EPSILON,
                  mc_trials=mc_trials, mc_seed=11)
    row = summarise(label, ev, res)
    row["m"] = m
    row["delta_v_mps"] = float(
        np.sum(np.linalg.norm(res.u, axis=1)) * S.DT / max(m, 1)
    )
    row["final_range_m"] = [
        float(np.linalg.norm(ev["positions"][i, -1] - S.TARGET)) for i in range(m)
    ]
    row["final_phase_angle_deg"] = [
        float(np.degrees(oracles[i].phase_angle(ev["positions"][i, -1]))) for i in range(m)
    ]
    return res, ev, row, agents, oracles


COLS = [
    "case",
    "m",
    "final_min_los_angle_deg",
    "final_fused_volume",
    "total_directional_radius",
    "D_O",
    "final_rmse",
    "final_setmembership_radius",
    "setmembership_coverage",
    "delta_v_mps",
    "outer_iterations",
    "solve_time_s",
]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--m", type=int, default=3, help="fleet size for the main comparison")
    ap.add_argument("--trials", type=int, default=400, help="Monte Carlo trials")
    ap.add_argument("--skip-ablation", action="store_true")
    args = ap.parse_args()

    print("=" * 100)
    print(f"Multi-Ego satellite proximity operations  (m = {args.m}, "
          f"dt = {S.DT} s, T = {S.HORIZON}, horizon {S.DT * S.HORIZON:.0f} s)")
    print("=" * 100)

    rows, arrays = [], {}

    # --- main comparison --------------------------------------------------
    for label, kw in S.CASES.items():
        res, ev, row, agents, oracles = run_case(args.m, label, kw, args.trials)
        rows.append(row)
        arrays[f"{label}_x"] = res.x
        arrays[f"{label}_u"] = res.u
        arrays[f"{label}_positions"] = ev["positions"]
        arrays[f"{label}_fused_volume"] = ev["fused_volume"]
        arrays[f"{label}_informativity"] = ev["informativity_ratio"]
        arrays[f"{label}_los_angle_deg"] = ev["min_los_angle_deg"]
        arrays[f"{label}_rmse"] = ev["estimation"].rmse
        arrays[f"{label}_sm_radius"] = ev["estimation"].sm_radius
        arrays[f"{label}_sm_volume"] = ev["estimation"].sm_volume
        arrays[f"{label}_objective"] = np.array(res.objective_history)
        arrays[f"{label}_step"] = np.array(res.step_history)

    print("\n" + "=" * 100)
    print(f"Main comparison (m = {args.m})")
    print_table(rows, COLS)

    # --- ablation over fleet size ----------------------------------------
    ablation_rows = []
    if not args.skip_ablation:
        print("\n" + "=" * 100)
        print("Ablation: fleet size")
        for m in range(1, S.START_STATES.shape[0] + 1):
            kw = S.CASES["network"] if m > 1 else S.CASES["own-set"]
            _, ev, row, _, _ = run_case(m, f"network-m{m}", kw, args.trials)
            ablation_rows.append(row)
            arrays[f"ablation_m{m}_positions"] = ev["positions"]
            arrays[f"ablation_m{m}_fused_volume"] = ev["fused_volume"]
            arrays[f"ablation_m{m}_rmse"] = ev["estimation"].rmse
            arrays[f"ablation_m{m}_sm_radius"] = ev["estimation"].sm_radius
        print("\n" + "=" * 100)
        print("Ablation summary  (m=1 has no network term, so it is the own-set planner)")
        print_table(ablation_rows, COLS)

    save("satellite", [{"main": rows, "ablation": ablation_rows,
                        "scenario": {
                            "dt": S.DT, "horizon": S.HORIZON,
                            "d_keepout": S.D_KEEPOUT, "d_collision": S.D_COLLISION,
                            "u_max": S.U_MAX, "epsilon": S.EPSILON,
                            "r_ref": S.R_REF, "a_depth": S.A_DEPTH,
                            "a_lateral": S.A_LATERAL, "w_illum": S.W_ILLUM,
                            "sun_direction": S.SUN_DIRECTION.tolist(),
                        }}], arrays)


if __name__ == "__main__":
    main()
