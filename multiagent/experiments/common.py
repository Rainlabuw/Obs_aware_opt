"""Shared metric extraction and result I/O for the experiment scripts."""

from __future__ import annotations

import json
import sys
from dataclasses import asdict, is_dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from maobs import (  # noqa: E402
    degree_of_observability,
    informativity_ratio,
    monte_carlo,
    network_shapes,
    total_directional_radius,
    volume_from_shape,
)

RESULTS = Path(__file__).resolve().parents[1] / "results"
RESULTS.mkdir(exist_ok=True)


def agent_positions(agents, x: np.ndarray) -> np.ndarray:
    """``(m, T+1, n_p)`` position array from the planner's ``(m, nx, T+1)`` states."""
    m = x.shape[0]
    T1 = x.shape[2]
    return np.stack(
        [np.stack([agents[i].position(x[i][:, t]) for t in range(T1)]) for i in range(m)]
    )


def bearings(positions: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Planar bearing of each agent from the Target, shape ``(m, T+1)``."""
    d = target[None, None, :] - positions
    return np.arctan2(d[..., 1], d[..., 0])


def pairwise_bearing_separation(positions: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Minimum absolute bearing separation over agent pairs, folded to [0, pi/2]."""
    phi = bearings(positions, target)
    m, T1 = phi.shape
    out = np.full(T1, np.pi / 2)
    for i in range(m):
        for j in range(i + 1, m):
            d = np.abs(np.angle(np.exp(1j * (phi[i] - phi[j]))))
            d = np.minimum(d, np.pi - d)  # 0 and pi are both "parallel"
            out = np.minimum(out, d)
    return out


def los_directions(positions: np.ndarray, target: np.ndarray) -> np.ndarray:
    d = target[None, None, :] - positions
    return d / np.linalg.norm(d, axis=-1, keepdims=True)


def min_los_angle(positions: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Smallest angle between any two agents' lines of sight, per time step."""
    u = los_directions(positions, target)
    m, T1, _ = u.shape
    out = np.full(T1, np.pi)
    for i in range(m):
        for j in range(i + 1, m):
            c = np.clip(np.sum(u[i] * u[j], axis=-1), -1.0, 1.0)
            ang = np.arccos(c)
            ang = np.minimum(ang, np.pi - ang)
            out = np.minimum(out, ang)
    return out if m > 1 else np.zeros(T1)


def evaluate(agents, oracles, x, target, truth=None, epsilon=None,
             mc_trials=400, mc_seed=0):
    """Full metric bundle for one planned solution."""
    positions = agent_positions(agents, x)
    fused, per_agent = network_shapes(oracles, positions)
    truth = target if truth is None else truth

    vol = np.array([volume_from_shape(Q) for Q in fused])
    ratio = np.array([informativity_ratio(per_agent[t], fused[t]) for t in range(len(fused))])
    tdr = total_directional_radius(fused)
    if epsilon is None:
        # Probe size large enough that no clamping occurs, so D_O is the affine
        # image of the total directional radius.  IMPORTANT: pass the *same*
        # epsilon for every case being compared -- a per-case epsilon makes the
        # D_O column meaningless across rows.
        epsilon = 5.0 * max(np.sqrt(np.max(np.diag(Q))) for Q in fused)
    do = degree_of_observability(fused, epsilon)
    max_radius = float(max(np.sqrt(np.max(np.diag(Q))) for Q in fused))
    if epsilon <= max_radius:
        print(f"  [warn] epsilon={epsilon:.4g} <= max fused radius {max_radius:.4g}: "
              "D_O is being clamped and is no longer affine in the directional radius")

    stats = monte_carlo(oracles, positions, np.asarray(truth, dtype=float),
                        trials=mc_trials, seed=mc_seed)

    return {
        "positions": positions,
        # Kept so D_O can be recomputed offline for any probe size without
        # re-planning -- see recompute_do.py.
        "fused_shapes": np.stack(fused),
        "max_fused_radius": max_radius,
        "fused_volume": vol,
        "informativity_ratio": ratio,
        "total_directional_radius": float(tdr),
        "degree_of_observability": float(do),
        "epsilon": float(epsilon),
        "mean_fused_volume": float(np.mean(vol)),
        "final_fused_volume": float(vol[-1]),
        "informative_fraction": float(np.mean(ratio < 1.0)),
        "min_los_angle_deg": np.degrees(min_los_angle(positions, np.asarray(target))),
        "estimation": stats,
    }


def summarise(label: str, ev: dict, planner=None) -> dict:
    s = ev["estimation"]
    row = {
        "case": label,
        "D_O": ev["degree_of_observability"],
        "total_directional_radius": ev["total_directional_radius"],
        "mean_fused_volume": ev["mean_fused_volume"],
        "final_fused_volume": ev["final_fused_volume"],
        "final_min_los_angle_deg": float(ev["min_los_angle_deg"][-1]),
        "informative_fraction": ev["informative_fraction"],
        "final_rmse": s.final_rmse,
        "mean_rmse": float(np.mean(s.rmse)),
        "final_setmembership_radius": s.final_sm_radius,
        "setmembership_coverage": s.coverage,
    }
    if planner is not None:
        row["outer_iterations"] = planner.outer_iterations
        row["converged"] = planner.converged
        row["solve_time_s"] = planner.solve_time
        row["subproblems"] = planner.subproblems_solved
    return row


def save(name: str, summary: list[dict], arrays: dict) -> None:
    with open(RESULTS / f"{name}.json", "w") as fh:
        json.dump(summary, fh, indent=2, default=_default)
    np.savez_compressed(RESULTS / f"{name}.npz", **arrays)
    print(f"\nsaved -> {RESULTS / (name + '.json')}")
    print(f"saved -> {RESULTS / (name + '.npz')}")


def _default(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    if is_dataclass(o):
        return asdict(o)
    if isinstance(o, (np.bool_, bool)):
        return bool(o)
    raise TypeError(type(o))


def print_table(rows: list[dict], cols: list[str] | None = None) -> None:
    if not rows:
        return
    cols = cols or list(rows[0].keys())
    widths = {c: max(len(c), *(len(_fmt(r.get(c))) for r in rows)) for c in cols}
    print("  ".join(c.ljust(widths[c]) for c in cols))
    print("  ".join("-" * widths[c] for c in cols))
    for r in rows:
        print("  ".join(_fmt(r.get(c)).ljust(widths[c]) for c in cols))


def _fmt(v) -> str:
    if v is None:
        return "-"
    if isinstance(v, bool) or isinstance(v, np.bool_):
        return "yes" if v else "no"
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    if isinstance(v, (float, np.floating)):
        a = abs(float(v))
        if a and (a < 1e-3 or a >= 1e5):
            return f"{v:.3e}"
        return f"{v:.4g}"
    return str(v)
