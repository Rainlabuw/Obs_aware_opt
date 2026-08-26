"""Discrete-time LTI agent models.

Two instantiations are provided, matching the two case studies:

``PlanarDoubleIntegrator``
    Planar point-mass agent, state ``[px, py, vx, vy]``, input ``[ax, ay]``.
    Used for the analytical toy study.

``ClohessyWiltshire``
    Relative translational dynamics of an Ego spacecraft in the (non-inertial)
    Hill frame of the Target, state ``[x, y, z, xdot, ydot, zdot]``, input
    ``[ux, uy, uz]``.  Discretised by matrix exponential exactly as in
    ``sat_rendezvous/scvx_obs_paper_run.py``.

Both expose the same interface so the planner is agnostic to which is used.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import expm

MU_EARTH = 3.986e14  # m^3 / s^2


def zoh_discretise(A: np.ndarray, B: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Exact zero-order-hold discretisation via the matrix exponential."""
    nx, nu = B.shape
    M = np.block([[A, B], [np.zeros((nu, nx)), np.zeros((nu, nu))]])
    D = expm(M * dt)
    return D[:nx, :nx], D[:nx, nx:]


@dataclass
class LinearAgent:
    """A discrete-time LTI agent ``x_{k+1} = Ad x_k + Bd u_k``, ``y_k = C x_k``."""

    Ad: np.ndarray
    Bd: np.ndarray
    C: np.ndarray
    dt: float
    n_pos: int
    name: str = "agent"

    @property
    def nx(self) -> int:
        return self.Ad.shape[0]

    @property
    def nu(self) -> int:
        return self.Bd.shape[1]

    def position(self, x: np.ndarray) -> np.ndarray:
        """Extract the position block from a state (or a state sequence)."""
        return np.asarray(x)[..., : self.n_pos]

    def rollout(self, x0: np.ndarray, u: np.ndarray) -> np.ndarray:
        """Propagate ``u`` (shape ``(nu, T)``) from ``x0``; returns ``(nx, T+1)``."""
        T = u.shape[1]
        x = np.zeros((self.nx, T + 1))
        x[:, 0] = x0
        for k in range(T):
            x[:, k + 1] = self.Ad @ x[:, k] + self.Bd @ u[:, k]
        return x


def planar_double_integrator(dt: float) -> LinearAgent:
    """Planar double integrator, discretised exactly."""
    A = np.block([[np.zeros((2, 2)), np.eye(2)], [np.zeros((2, 2)), np.zeros((2, 2))]])
    B = np.vstack([np.zeros((2, 2)), np.eye(2)])
    Ad, Bd = zoh_discretise(A, B, dt)
    C = np.hstack([np.eye(2), np.zeros((2, 2))])
    return LinearAgent(Ad=Ad, Bd=Bd, C=C, dt=dt, n_pos=2, name="planar-double-integrator")


def clohessy_wiltshire(dt: float, a_orbit: float = 6.9e6) -> LinearAgent:
    """Clohessy-Wiltshire relative dynamics for a circular reference orbit.

    ``a_orbit`` is the semi-major axis of the Target's circular orbit in metres;
    the default 6900 km matches the single-agent study.
    """
    n = np.sqrt(MU_EARTH / a_orbit**3)
    A = np.array(
        [
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
            [3 * n**2, 0, 0, 0, 2 * n, 0],
            [0, 0, 0, -2 * n, 0, 0],
            [0, 0, -(n**2), 0, 0, 0],
        ]
    )
    B = np.vstack([np.zeros((3, 3)), np.eye(3)])
    Ad, Bd = zoh_discretise(A, B, dt)
    C = np.hstack([np.eye(3), np.zeros((3, 3))])
    agent = LinearAgent(Ad=Ad, Bd=Bd, C=C, dt=dt, n_pos=3, name="clohessy-wiltshire")
    agent.mean_motion = n  # type: ignore[attr-defined]
    return agent
