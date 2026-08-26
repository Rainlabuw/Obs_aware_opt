"""Sanity checks on the oracles, metrics and the analytical orthogonality claim."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from maobs import (  # noqa: E402
    MonocularPoseOracle,
    RangeBearingOracle,
    clohessy_wiltshire,
    degree_of_observability,
    informativity_ratio,
    planar_double_integrator,
    total_directional_radius,
    volume_from_shape,
)
from maobs.observability import fuse, support_radius  # noqa: E402


def check(name: str, ok: bool, detail: str = "") -> bool:
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))
    return ok


def main() -> int:
    ok = True
    rng = np.random.default_rng(0)

    # --- dynamics ---------------------------------------------------------
    pdi = planar_double_integrator(0.5)
    x = pdi.rollout(np.array([1.0, 2.0, 0.5, -0.5]), np.zeros((2, 4)))
    ok &= check("planar drift", np.allclose(x[:2, -1], [1 + 0.5 * 2, 2 - 0.5 * 2]),
                f"end pos {x[:2, -1]}")

    cw = clohessy_wiltshire(5.0)
    n = cw.mean_motion
    # Over one step the position-velocity block is dt*I plus the O(n dt^2)
    # Coriolis cross-coupling that is the whole point of CW dynamics.
    ok &= check("CW position-velocity block is dt*I to O(n dt)",
                np.allclose(np.diag(cw.Ad[:3, 3:]), 5.0, atol=1e-3)
                # xddot = ... + 2 n ydot  =>  Ad[0,4] ~ (2n) dt^2 / 2 = n dt^2
                and np.isclose(cw.Ad[0, 4], n * 5.0**2, rtol=5e-3),
                f"coupling {cw.Ad[0, 4]:.5f}, n dt^2 = {n * 25:.5f}")
    ok &= check("CW period is one orbit", np.isclose(2 * np.pi / n, 5704, rtol=1e-3),
                f"{2 * np.pi / n:.0f} s")

    # --- oracle: convexity of radii along a random segment ----------------
    orc = MonocularPoseOracle(target=np.zeros(3), sun_direction=np.array([1.0, 0, 0]))
    pa, pb = np.array([-25.0, 8.0, 3.0]), np.array([12.0, -20.0, 6.0])
    lams = np.linspace(0, 1, 21)
    conv_ok = True
    for j in range(3):
        vals = np.array([orc.radii(pa + l * (pb - pa))[0][j] for l in lams])
        chord = vals[0] + lams * (vals[-1] - vals[0])
        conv_ok &= bool(np.all(vals <= chord + 1e-9))
    ok &= check("monocular radii are convex along a segment", conv_ok)

    # --- oracle: geometry -------------------------------------------------
    p_good = orc.p_star
    p_bad = -orc.p_star
    ok &= check("best-illumination position has smaller set",
                orc.size(p_good) < orc.size(p_bad),
                f"{orc.size(p_good):.3f} vs {orc.size(p_bad):.3f}")
    ok &= check("depth is the weak axis", orc.radii(p_good)[0][0] > orc.radii(p_good)[0][1],
                f"sigma = {np.round(orc.radii(p_good)[0], 3)}")
    ok &= check("phase angle at p_star is zero", abs(orc.phase_angle(p_good)) < 1e-9)

    # --- support radius consistency --------------------------------------
    Q = orc.shape(np.array([-20.0, 5.0, 0.0]))
    nu = np.array([0.0, 1.0, 0.0])
    ok &= check("support radius matches sqrt of quadratic form",
                np.isclose(support_radius(Q, nu), np.sqrt(Q[1, 1])))

    # --- analytical orthogonality result ----------------------------------
    # For two co-centred planar ellipses with radii (a,b) at bearings phi1,phi2,
    #   det(Q1^-1 + Q2^-1) = (alpha+beta)^2 - (alpha-beta)^2 cos^2(dphi),
    # with alpha=1/a^2, beta=1/b^2.  Maximised (fused volume minimised) at
    # dphi = pi/2, and the gain vanishes when a == b.
    a, b = 0.5, 2.5
    alpha, beta = 1 / a**2, 1 / b**2

    def planar_info(phi):
        R = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
        return R @ np.diag([alpha, beta]) @ R.T

    err = 0.0
    for _ in range(200):
        phi1, phi2 = rng.uniform(0, np.pi, 2)
        lhs = np.linalg.det(planar_info(phi1) + planar_info(phi2))
        rhs = (alpha + beta) ** 2 - (alpha - beta) ** 2 * np.cos(phi1 - phi2) ** 2
        err = max(err, abs(lhs - rhs))
    ok &= check("closed form for fused information determinant", err < 1e-8, f"max err {err:.2e}")

    dphis = np.linspace(0, np.pi / 2, 91)
    dets = [np.linalg.det(planar_info(0.0) + planar_info(d)) for d in dphis]
    ok &= check("fused information is maximised at orthogonal bearings",
                abs(dphis[int(np.argmax(dets))] - np.pi / 2) < 1e-6)

    gain = np.sqrt(max(dets) / min(dets))
    predicted = (alpha + beta) / (2 * np.sqrt(alpha * beta))
    ok &= check("volume gain matches (alpha+beta)/(2 sqrt(alpha beta))",
                np.isclose(gain, predicted, rtol=1e-6),
                f"{gain:.4f} vs {predicted:.4f}")

    iso = 1.0
    dets_iso = [np.linalg.det(np.eye(2) / iso + planar_info_iso(d)) for d in dphis]
    ok &= check("no orthogonality gain for isotropic sets",
                np.ptp(dets_iso) < 1e-9)

    # --- informativity ----------------------------------------------------
    o1 = RangeBearingOracle(target=np.zeros(2), sigma_r=0.3, sigma_phi=0.05)
    Q1 = o1.shape(np.array([-15.0, 0.0]))
    Q2 = o1.shape(np.array([0.0, -15.0]))
    Qf = fuse([np.linalg.inv(Q1), np.linalg.inv(Q2)])
    ratio = informativity_ratio([Q1, Q2], Qf)
    ok &= check("orthogonal pair is informative", ratio < 1.0, f"ratio {ratio:.4f}")

    Q3 = o1.shape(np.array([-15.0, 0.3]))
    Qf2 = fuse([np.linalg.inv(Q1), np.linalg.inv(Q3)])
    ok &= check("near-parallel pair is much less informative",
                informativity_ratio([Q1, Q3], Qf2) > ratio,
                f"{informativity_ratio([Q1, Q3], Qf2):.4f} > {ratio:.4f}")

    # --- D_O is affine-decreasing in total directional radius -------------
    shapes = [orc.shape(np.array([-20.0 - 2 * k, 4.0, 1.0])) for k in range(5)]
    eps = 50.0
    do = degree_of_observability(shapes, eps)
    tdr = total_directional_radius(shapes)
    expected = 2.0 * (len(shapes) * 3 * eps - tdr)
    ok &= check("D_O = 2(n_z T eps - total directional radius)",
                np.isclose(do, expected), f"{do:.4f} vs {expected:.4f}")

    print()
    print("ALL PASS" if ok else "SOME CHECKS FAILED")
    return 0 if ok else 1


def planar_info_iso(phi):
    R = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
    return R @ np.diag([1.0, 1.0]) @ R.T


if __name__ == "__main__":
    raise SystemExit(main())
