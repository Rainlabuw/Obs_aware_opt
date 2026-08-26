"""Planar toy study: does the planner recover the orthogonal approach?

Two planar double-integrator robots with range-bearing sensors approach a
stationary Target from nearly the same bearing -- the pathological
non-informative configuration of Definition (Network Informativity).  We
compare three planners:

    agnostic   alpha_obs = 0                 (primary task only)
    own-set    alpha_obs > 0, lambda_N = 0   (each robot shrinks its own set,
                                              blind to its neighbour)
    network    alpha_obs > 0, lambda_N > 0   (full Algorithm 1)

and check the resulting bearing separation against the analytical prediction
that the fused set is smallest at 90 degrees.

A closed-form sweep is also produced: two agents pinned on the keep-out ring at
bearing separation ``dphi``, fused volume as a function of ``dphi``.  This is
the analytical sanity check referred to in the manuscript's corollary.
"""

from __future__ import annotations

import numpy as np

from common import evaluate, print_table, save, summarise  # noqa: E402

from maobs import PlannerConfig, RangeBearingOracle, plan, planar_double_integrator  # noqa: E402
from maobs.observability import fuse, volume_from_shape  # noqa: E402

DT = 0.5
HORIZON = 40
TARGET = np.array([0.0, 0.0])
D_KEEPOUT = 10.0
SIGMA_R = 0.30
SIGMA_PHI = 0.06
# Probe size for the degree of observability.  Fixed across cases so the D_O
# column is comparable; comfortably larger than any fused set radius here, so
# no clamping occurs and D_O is exactly affine in the total directional radius.
EPSILON = 1.0

X0 = np.array(
    [
        [-32.0, -5.0, 0.0, 0.0],
        [-32.0, +5.0, 0.0, 0.0],
    ]
)
X_DES = np.zeros((2, 4))


def base_config(**kw) -> PlannerConfig:
    cfg = PlannerConfig(
        horizon=HORIZON,
        w_state=2e-3,
        w_control=2e-2,
        w_terminal=2e-2,
        Q_state=np.diag([1.0, 1.0, 4.0, 4.0]),
        R_control=np.eye(2),
        d_keepout=D_KEEPOUT,
        d_collision=3.0,
        u_max=2.0,
        penalty=5e2,
        trust_radius=6.0,
        trust_max=30.0,
        max_outer=60,
        tol=5e-3,
        obj_tol=2e-4,
        alpha_obs=0.0,
        lambda_net=0.0,
    )
    for k, v in kw.items():
        setattr(cfg, k, v)
    return cfg


def make(m: int = 2):
    agents = [planar_double_integrator(DT) for _ in range(m)]
    oracles = [
        RangeBearingOracle(target=TARGET, sigma_r=SIGMA_R, sigma_phi=SIGMA_PHI)
        for _ in range(m)
    ]
    return agents, oracles


def analytic_sweep(radius: float = D_KEEPOUT, n: int = 181) -> dict:
    """Fused volume vs bearing separation for two agents pinned on the ring."""
    orc = RangeBearingOracle(target=TARGET, sigma_r=SIGMA_R, sigma_phi=SIGMA_PHI)
    dphis = np.linspace(0.0, np.pi / 2, n)
    vols, ratios = [], []
    for d in dphis:
        p1 = TARGET + radius * np.array([np.cos(0.0), np.sin(0.0)])
        p2 = TARGET + radius * np.array([np.cos(d), np.sin(d)])
        Q1, Q2 = orc.shape(p1), orc.shape(p2)
        Qf = fuse([np.linalg.inv(Q1), np.linalg.inv(Q2)])
        vols.append(volume_from_shape(Qf))
        ratios.append(volume_from_shape(Qf) / min(volume_from_shape(Q1), volume_from_shape(Q2)))
    vols = np.array(vols)
    alpha, beta = 1.0 / SIGMA_R**2, 1.0 / (radius * SIGMA_PHI) ** 2
    return {
        "dphi_deg": np.degrees(dphis),
        "fused_volume": vols,
        "informativity_ratio": np.array(ratios),
        "argmin_deg": float(np.degrees(dphis[int(np.argmin(vols))])),
        "predicted_gain": float((alpha + beta) / (2.0 * np.sqrt(alpha * beta))),
        "observed_gain": float(vols.max() / vols.min()),
        "anisotropy": float(max(SIGMA_R, radius * SIGMA_PHI) / min(SIGMA_R, radius * SIGMA_PHI)),
    }


CASES = {
    "agnostic": dict(alpha_obs=0.0, lambda_net=0.0),
    "own-set": dict(alpha_obs=3.0, lambda_net=0.0),
    "network": dict(alpha_obs=3.0, lambda_net=1.5),
}


def main() -> None:
    print("=" * 78)
    print("Planar two-robot range-bearing study")
    print("=" * 78)

    sweep = analytic_sweep()
    print("\nAnalytic sweep (two agents pinned on the keep-out ring):")
    print(f"  fused volume minimised at dphi = {sweep['argmin_deg']:.2f} deg")
    print(f"  set anisotropy at r = {D_KEEPOUT:.0f} m : {sweep['anisotropy']:.3f}")
    print(f"  predicted volume gain  (alpha+beta)/(2 sqrt(alpha beta)) = "
          f"{sweep['predicted_gain']:.4f}")
    print(f"  observed  volume gain  max/min                           = "
          f"{sweep['observed_gain']:.4f}")

    rows, arrays = [], {}
    arrays["sweep_dphi_deg"] = sweep["dphi_deg"]
    arrays["sweep_fused_volume"] = sweep["fused_volume"]
    arrays["sweep_informativity_ratio"] = sweep["informativity_ratio"]

    for label, kw in CASES.items():
        agents, oracles = make(2)
        cfg = base_config(**kw)
        print(f"\n--- planning: {label} "
              f"(alpha_obs={cfg.alpha_obs}, lambda_N={cfg.lambda_net}) ---")
        res = plan(agents, oracles, X0, X_DES, TARGET, cfg)
        print(f"    outer iterations {res.outer_iterations}, "
              f"converged={res.converged} ({res.convergence_reason}), "
              f"{res.solve_time:.1f} s, {res.subproblems_solved} subproblems, "
              f"{res.inaccurate_solves} inaccurate, {res.failed_solves} failed")

        ev = evaluate(agents, oracles, res.x, TARGET, epsilon=EPSILON,
                      mc_trials=400, mc_seed=7)
        rows.append(summarise(label, ev, res))
        arrays[f"{label}_x"] = res.x
        arrays[f"{label}_u"] = res.u
        arrays[f"{label}_positions"] = ev["positions"]
        arrays[f"{label}_fused_volume"] = ev["fused_volume"]
        arrays[f"{label}_los_angle_deg"] = ev["min_los_angle_deg"]
        arrays[f"{label}_rmse"] = ev["estimation"].rmse
        arrays[f"{label}_sm_radius"] = ev["estimation"].sm_radius
        arrays[f"{label}_objective"] = np.array(res.objective_history)

    print("\n" + "=" * 78)
    print_table(
        rows,
        [
            "case",
            "final_min_los_angle_deg",
            "mean_fused_volume",
            "final_fused_volume",
            "total_directional_radius",
            "D_O",
            "final_rmse",
            "final_setmembership_radius",
            "setmembership_coverage",
            "outer_iterations",
            "solve_time_s",
        ],
    )

    summary = {"cases": rows, "analytic_sweep": {k: v for k, v in sweep.items()
                                                 if not isinstance(v, np.ndarray)}}
    save("planar", [summary], arrays)


if __name__ == "__main__":
    main()
