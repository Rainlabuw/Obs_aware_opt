"""Scenario definition for the multi-Ego satellite proximity-operations study.

A fleet of Ego spacecraft in Clohessy-Wiltshire relative motion approaches an
uncooperative Target at the Hill-frame origin.  Each Ego carries a monocular
camera feeding a learned 6-DOF pose estimator, modelled by
``MonocularPoseOracle``: the uncertainty ellipsoid is elongated along the
line of sight (depth is the weak direction), grows with range, and is inflated
away from the best-illumination viewing position.

The fleet starts on the sunward side of the Target -- backlit, badly
conditioned, and bunched together -- which is the configuration the planner has
to fix.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from maobs import MonocularPoseOracle, PlannerConfig, clohessy_wiltshire  # noqa: E402

DT = 7.5  # s
HORIZON = 40  # -> 300 s, about 5 % of the orbit period
A_ORBIT = 6.9e6  # m
TARGET = np.zeros(3)
SUN_DIRECTION = np.array([1.0, 0.0, 0.0])  # direction sunlight propagates

D_KEEPOUT = 15.0  # m
D_COLLISION = 6.0  # m
U_MAX = 0.010  # m/s^2

# Oracle parameters (metres).  At the reference range with perfect illumination
# the depth radius is 1.2 m and the lateral radius 0.15 m: an 8:1 anisotropy,
# which is the regime a monocular pose head actually operates in.
R_REF = 20.0
A_DEPTH = 1.2
A_LATERAL = 0.15
# Illumination coupling.  Calibrated so that the illumination penalty and the
# range term are of comparable magnitude over the operating box: with a much
# larger value every Ego crowds into the same well-lit spot and the geometric
# diversity the network term is supposed to create is swamped.  That tension is
# real and is discussed in the results.
W_ILLUM = 0.10

# Probe size for D_O.  Fixed across every case and every fleet size so the
# column is comparable; larger than any fused radius encountered, so no clamping.
EPSILON = 5.0

# Ego initial states, all on the sunward (+x) side and clustered in bearing.
START_STATES = np.array(
    [
        [58.0, 18.0, 6.0, 0.0, 0.0, 0.0],
        [56.0, -16.0, -7.0, 0.0, 0.0, 0.0],
        [60.0, 4.0, 20.0, 0.0, 0.0, 0.0],
        [54.0, -2.0, -21.0, 0.0, 0.0, 0.0],
    ]
)


def make(m: int):
    """Agents, oracles, initial and desired states for a fleet of ``m`` Egos."""
    if not 1 <= m <= START_STATES.shape[0]:
        raise ValueError(f"m must be in 1..{START_STATES.shape[0]}")
    agents = [clohessy_wiltshire(DT, A_ORBIT) for _ in range(m)]
    oracles = [
        MonocularPoseOracle(
            target=TARGET,
            sun_direction=SUN_DIRECTION,
            r_ref=R_REF,
            a_depth=A_DEPTH,
            a_lateral=A_LATERAL,
            w_illum=W_ILLUM,
        )
        for _ in range(m)
    ]
    x0 = START_STATES[:m].copy()
    x_des = np.zeros((m, 6))
    return agents, oracles, x0, x_des


def config(**kw) -> PlannerConfig:
    cfg = PlannerConfig(
        horizon=HORIZON,
        # Balanced so that the primary task (close to the keep-out ring) is met
        # by every case including the estimation-agnostic baseline, leaving the
        # estimation terms to decide *where* on the ring each Ego parks.
        w_state=0.02,
        w_control=5e4,
        w_terminal=2.0,
        Q_state=np.diag([1.0, 1.0, 1.0, 2e3, 2e3, 2e3]),
        R_control=np.eye(3),
        d_keepout=D_KEEPOUT,
        d_collision=D_COLLISION,
        u_max=U_MAX,
        penalty=2e2,
        trust_radius=10.0,
        trust_max=60.0,
        max_outer=60,
        tol=1e-2,
        obj_tol=2e-4,
        alpha_obs=0.0,
        lambda_net=0.0,
    )
    for k, v in kw.items():
        setattr(cfg, k, v)
    return cfg


CASES = {
    "agnostic": dict(alpha_obs=0.0, lambda_net=0.0),
    "own-set": dict(alpha_obs=6.0, lambda_net=0.0),
    "network": dict(alpha_obs=6.0, lambda_net=20.0),
}
