"""Figure generation for the manuscript.

Deliberately separate from the simulation: ``maobs`` and every ``run_*.py``
script are visualisation-free and write only arrays.  This script reads those
arrays back and renders PNGs into ``figures/``.  It is the only place
matplotlib is imported.

    python figures.py            # all figures
    python figures.py planar     # just the planar ones
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
RESULTS = ROOT / "results"
FIGURES = ROOT / "figures"
FIGURES.mkdir(exist_ok=True)

plt.rcParams.update({
    "font.size": 11,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "figure.dpi": 160,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.03,
    "legend.frameon": False,
})

STYLE = {
    "agnostic": dict(color="#8c8c8c", ls="--", lw=2.0, label="estimation-agnostic"),
    "own-set": dict(color="#1f77b4", ls="-.", lw=2.0, label="own-set aware ($\\lambda_N=0$)"),
    "network": dict(color="#d62728", ls="-", lw=2.4, label="network aware"),
}
AGENT_COLORS = ["#d62728", "#1f77b4", "#2ca02c", "#9467bd"]


def _load(name: str):
    npz = np.load(RESULTS / f"{name}.npz")
    with open(RESULTS / f"{name}.json") as fh:
        meta = json.load(fh)
    return npz, meta


def _ellipse_points(Q2, center, n=80):
    """Boundary of the 2-D ellipse ``E(center, Q2)``."""
    w, V = np.linalg.eigh(Q2)
    th = np.linspace(0, 2 * np.pi, n)
    circle = np.stack([np.cos(th), np.sin(th)])
    return (V @ (np.sqrt(np.maximum(w, 0))[:, None] * circle)).T + center


# ---------------------------------------------------------------------------
# planar
# ---------------------------------------------------------------------------


def planar() -> None:
    from maobs import RangeBearingOracle
    import run_planar as RP

    d, meta = _load("planar")

    # (a) analytic sweep, (b) trajectories, (c) final uncertainty geometry
    fig, axes = plt.subplots(1, 3, figsize=(15.0, 4.3), constrained_layout=True)

    ax = axes[0]
    ax.plot(d["sweep_dphi_deg"], d["sweep_fused_volume"], color="#d62728", lw=2.4)
    imin = int(np.argmin(d["sweep_fused_volume"]))
    ax.axvline(d["sweep_dphi_deg"][imin], color="k", ls=":", lw=1.2)
    ax.annotate(f"$\\Delta\\varphi^\\star = {d['sweep_dphi_deg'][imin]:.0f}^\\circ$",
                xy=(d["sweep_dphi_deg"][imin], d["sweep_fused_volume"][imin]),
                xytext=(-95, 24), textcoords="offset points",
                arrowprops=dict(arrowstyle="->", lw=1.0))
    ax.set_xlabel("bearing separation $\\Delta\\varphi$ [deg]")
    ax.set_ylabel("$\\mathrm{Vol}(\\mathcal{Y}_{\\mathbf{z}})$")
    ax.set_title("(a) analytic: fused set vs. bearing separation")

    ax = axes[1]
    orc = RangeBearingOracle(target=RP.TARGET, sigma_r=RP.SIGMA_R, sigma_phi=RP.SIGMA_PHI)
    th = np.linspace(0, 2 * np.pi, 200)
    ax.plot(RP.D_KEEPOUT * np.cos(th), RP.D_KEEPOUT * np.sin(th), color="k", lw=1.0, ls=":",
            label="keep-out")
    for case in ("agnostic", "network"):
        P = d[f"{case}_positions"]
        st = STYLE[case]
        for i in range(P.shape[0]):
            ax.plot(P[i, :, 0], P[i, :, 1], color=st["color"], ls=st["ls"], lw=st["lw"],
                    label=st["label"] if i == 0 else None)
            ax.plot(P[i, -1, 0], P[i, -1, 1], "o", color=st["color"], ms=7,
                    markeredgecolor="k", markeredgewidth=0.5, zorder=5)
            # line of sight at the final state
            ax.plot([P[i, -1, 0], RP.TARGET[0]], [P[i, -1, 1], RP.TARGET[1]],
                    color=st["color"], lw=0.8, alpha=0.45)
    ax.plot(d["agnostic_positions"][:, 0, 0], d["agnostic_positions"][:, 0, 1],
            "ks", ms=6, label="start", zorder=5)
    ax.plot(*RP.TARGET, "k*", ms=15, label="Target", zorder=6)
    ax.set_aspect("equal")
    ax.set_xlabel("$x$ [m]")
    ax.set_ylabel("$y$ [m]")
    ax.set_title("(b) planned trajectories")
    ax.legend(loc="upper left", fontsize=8, frameon=True, framealpha=0.92,
              edgecolor="none", borderpad=0.4)

    # (c) final uncertainty sets, true scale, centred on the Target
    ax = axes[2]
    for case in ("agnostic", "network"):
        P = d[f"{case}_positions"]
        st = STYLE[case]
        infos = []
        for i in range(P.shape[0]):
            Q = orc.shape(P[i, -1])
            infos.append(np.linalg.inv(Q))
            E = _ellipse_points(Q, RP.TARGET)
            ax.plot(E[:, 0], E[:, 1], color=st["color"], lw=1.0, ls=":", alpha=0.7)
        Qf = np.linalg.inv(sum(infos))
        E = _ellipse_points(Qf, RP.TARGET)
        ax.fill(E[:, 0], E[:, 1], color=st["color"], alpha=0.28)
        ax.plot(E[:, 0], E[:, 1], color=st["color"], lw=2.2, label=st["label"])
    ax.plot(*RP.TARGET, "k*", ms=13)
    ax.set_aspect("equal")
    ax.set_xlabel("$z_x$ [m]")
    ax.set_ylabel("$z_y$ [m]")
    ax.set_title("(c) final sets: agents (dotted) and fused (filled)")
    ax.legend(loc="upper right", fontsize=8.5)

    fig.savefig(FIGURES / "planar_geometry.png")
    plt.close(fig)

    # (c) metrics vs time
    fig, axes = plt.subplots(1, 3, figsize=(14.5, 3.9), constrained_layout=True)
    t = np.arange(d["agnostic_los_angle_deg"].size) * RP.DT
    for case in STYLE:
        st = STYLE[case]
        axes[0].plot(t, d[f"{case}_los_angle_deg"], **st)
        axes[1].plot(t, d[f"{case}_fused_volume"], **st)
        axes[2].plot(t, d[f"{case}_rmse"], **st)
    axes[0].axhline(90, color="k", ls=":", lw=1.0)
    axes[0].set_ylabel("bearing separation [deg]")
    axes[0].set_title("(a) network geometry")
    axes[1].set_ylabel("$\\mathrm{Vol}(\\mathcal{Y}_{\\mathbf{z}})$")
    axes[1].set_yscale("log")
    axes[1].set_title("(b) fused set volume")
    axes[2].set_ylabel("target position RMSE [m]")
    axes[2].set_yscale("log")
    axes[2].set_title("(c) estimation error")
    for a in axes:
        a.set_xlabel("time [s]")
    axes[0].legend(fontsize=9, loc="center right")
    fig.savefig(FIGURES / "planar_metrics.png")
    plt.close(fig)
    print(f"wrote {FIGURES/'planar_geometry.png'}")
    print(f"wrote {FIGURES/'planar_metrics.png'}")


# ---------------------------------------------------------------------------
# satellite
# ---------------------------------------------------------------------------


def satellite() -> None:
    import scenario_satellite as S

    d, meta = _load("satellite")
    main_rows = meta[0]["main"]
    ablation_rows = meta[0]["ablation"]
    by_case = {r["case"]: r for r in main_rows}

    # --- 3-D trajectories + top-down geometry ---------------------------
    fig = plt.figure(figsize=(15.5, 4.8), constrained_layout=True)
    lim = 65.0
    for j, case in enumerate(("agnostic", "network")):
        ax = fig.add_subplot(1, 3, j + 1, projection="3d")
        P = d[f"{case}_positions"]
        _keepout_sphere(ax, S.D_KEEPOUT)
        for i in range(P.shape[0]):
            c = AGENT_COLORS[i]
            ax.plot(P[i, :, 0], P[i, :, 1], P[i, :, 2], color=c, lw=2.2, label=f"Ego {i+1}")
            ax.scatter(*P[i, 0], color=c, marker="s", s=34)
            ax.scatter(*P[i, -1], color=c, marker="o", s=56, edgecolor="k", linewidth=0.6)
            ax.plot([P[i, -1, 0], 0], [P[i, -1, 1], 0], [P[i, -1, 2], 0],
                    color=c, lw=0.8, alpha=0.4)
        ax.scatter(0, 0, 0, color="k", marker="*", s=150)
        ax.quiver(-34, 0, 0, 1, 0, 0, length=16, color="darkorange", lw=2.5,
                  arrow_length_ratio=0.3)
        ax.set_xlim(-35, lim)
        ax.set_ylim(-32, 32)
        ax.set_zlim(-32, 32)
        ax.set_box_aspect((100, 64, 64))
        ax.set_xlabel("$x$ [m]", labelpad=-4)
        ax.set_ylabel("$y$ [m]", labelpad=-4)
        ax.set_zlabel("$z$ [m]", labelpad=-4)
        ax.tick_params(labelsize=7, pad=-2)
        ax.set_title(f"({'ab'[j]}) {STYLE[case]['label']}\n"
                     f"$\\mathrm{{Vol}}(\\mathcal{{Y}}_{{\\mathbf{{z}}}})$ = "
                     f"{by_case[case]['final_fused_volume']:.3g} m$^3$", fontsize=10)
        ax.view_init(elev=30, azim=-62)
        if j == 0:
            ax.legend(fontsize=8, loc="upper left", bbox_to_anchor=(-0.08, 0.98))

    # (c) top-down projection of the final configuration -- this is where the
    # bearing spread is actually legible.
    ax = fig.add_subplot(1, 3, 3)
    th = np.linspace(0, 2 * np.pi, 200)
    ax.plot(S.D_KEEPOUT * np.cos(th), S.D_KEEPOUT * np.sin(th), "k:", lw=1.0,
            label="keep-out")
    for case in ("agnostic", "network"):
        P = d[f"{case}_positions"]
        st = STYLE[case]
        for i in range(P.shape[0]):
            ax.plot([P[i, -1, 0], 0], [P[i, -1, 1], 0], color=st["color"],
                    ls=st["ls"], lw=1.6, label=st["label"] if i == 0 else None)
            ax.plot(P[i, -1, 0], P[i, -1, 1], "o", color=st["color"], ms=7,
                    markeredgecolor="k", markeredgewidth=0.5)
    ax.plot(0, 0, "k*", ms=15)
    ax.set_xlim(-24, 20)
    ax.set_ylim(-18, 18)
    ax.annotate("", xy=(-14.5, 13.5), xytext=(-22, 13.5),
                arrowprops=dict(arrowstyle="-|>", color="darkorange", lw=2.5))
    ax.text(-22, 14.6, "sun", color="darkorange", fontsize=9)
    ax.set_aspect("equal")
    ax.set_xlabel("$x$ [m]")
    ax.set_ylabel("$y$ [m]")
    ax.set_title("(c) final lines of sight, top-down", fontsize=10)
    ax.legend(fontsize=8, loc="upper right")

    fig.savefig(FIGURES / "satellite_trajectories.png")
    plt.close(fig)

    # --- metrics vs time -------------------------------------------------
    fig, axes = plt.subplots(1, 4, figsize=(18.0, 3.9), constrained_layout=True)
    t = np.arange(d["agnostic_fused_volume"].size) * S.DT
    for case in STYLE:
        st = STYLE[case]
        axes[0].plot(t, d[f"{case}_los_angle_deg"], **st)
        axes[1].plot(t, d[f"{case}_fused_volume"], **st)
        axes[2].plot(t, d[f"{case}_rmse"], **st)
        axes[3].plot(t, d[f"{case}_sm_radius"], **st)
    axes[0].set_ylabel("min. LOS separation [deg]")
    axes[0].set_title("(a) network geometry")
    axes[1].set_ylabel("$\\mathrm{Vol}(\\mathcal{Y}_{\\mathbf{z}})$ [m$^3$]")
    axes[1].set_yscale("log")
    axes[1].set_title("(b) fused set volume")
    axes[2].set_ylabel("Target RMSE [m]")
    axes[2].set_yscale("log")
    axes[2].set_title("(c) Gaussian fusion filter")
    axes[3].set_ylabel("guaranteed radius [m]")
    axes[3].set_yscale("log")
    axes[3].set_title("(d) set-membership bound")
    for a in axes:
        a.set_xlabel("time [s]")
    axes[0].legend(fontsize=9)
    fig.savefig(FIGURES / "satellite_metrics.png")
    plt.close(fig)

    # --- ablation + convergence -----------------------------------------
    fig, axes = plt.subplots(1, 3, figsize=(14.5, 3.9), constrained_layout=True)
    ms = [r["m"] for r in ablation_rows]

    ax = axes[0]
    ax.plot(ms, [r["final_fused_volume"] for r in ablation_rows], "o-", color="#d62728", lw=2.2)
    ax.set_yscale("log")
    ax.set_xlabel("number of Ego agents $m$")
    ax.set_ylabel("$\\mathrm{Vol}(\\mathcal{Y}_{\\mathbf{z}})$ [m$^3$]")
    ax.set_xticks(ms)
    ax.set_title("(a) ablation: fused set volume")

    ax = axes[1]
    ax.plot(ms, [r["final_rmse"] for r in ablation_rows], "o-", color="#1f77b4", lw=2.2,
            label="Gaussian RMSE")
    ax.plot(ms, [r["final_setmembership_radius"] for r in ablation_rows], "s--",
            color="#2ca02c", lw=2.2, label="set-membership bound")
    ax.set_yscale("log")
    ax.set_xlabel("number of Ego agents $m$")
    ax.set_ylabel("final error [m]")
    ax.set_xticks(ms)
    ax.set_title("(b) ablation: estimation error")
    ax.legend(fontsize=9)

    # The three cases carry different weights, so their raw objectives are not
    # comparable in absolute value.  Normalising by the total decrease makes the
    # convergence *rate* comparable, which is what the panel is about.
    ax = axes[2]
    for case in STYLE:
        obj = d[f"{case}_objective"]
        rel = (obj - obj[-1]) / max(obj[0] - obj[-1], 1e-12)
        ax.semilogy(np.arange(1, obj.size + 1), np.maximum(rel, 1e-6), **STYLE[case])
    ax.set_xlabel("outer iteration $k$")
    ax.set_ylabel("$(J_k - J_\\infty)\\,/\\,(J_0 - J_\\infty)$")
    ax.set_title("(c) convergence of Algorithm 1")
    ax.legend(fontsize=9)

    fig.savefig(FIGURES / "satellite_ablation.png")
    plt.close(fig)

    print(f"wrote {FIGURES/'satellite_trajectories.png'}")
    print(f"wrote {FIGURES/'satellite_metrics.png'}")
    print(f"wrote {FIGURES/'satellite_ablation.png'}")


def _keepout_sphere(ax, radius: float) -> None:
    u = np.linspace(0, 2 * np.pi, 40)
    v = np.linspace(0, np.pi, 20)
    ax.plot_surface(
        radius * np.outer(np.cos(u), np.sin(v)),
        radius * np.outer(np.sin(u), np.sin(v)),
        radius * np.outer(np.ones_like(u), np.cos(v)),
        color="steelblue", alpha=0.18, linewidth=0,
    )


def main() -> None:
    which = sys.argv[1:] or ["planar", "satellite"]
    if "planar" in which and (RESULTS / "planar.npz").exists():
        planar()
    if "satellite" in which and (RESULTS / "satellite.npz").exists():
        satellite()


if __name__ == "__main__":
    main()
