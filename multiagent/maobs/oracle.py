"""Set-valued output uncertainty oracles.

An oracle is the point-to-set map ``Y : X -> 2^{R^{n_z}}`` of the paper,
restricted here to ellipsoids

.. math::
    \\mathcal{E}(\\bm z, Q(\\bm x))
      = \\{ \\bm z' : (\\bm z' - \\bm z)^\\top Q(\\bm x)^{-1} (\\bm z' - \\bm z) \\le 1 \\},

so ``Q = V diag(sigma^2) V^T`` with ``sigma`` the principal semi-axis lengths
("set radii") and ``V`` the principal axes.  Every oracle here is written so
that each principal radius is a **convex** function of the agent position,
which is Assumption (Directional Convexity) in the manuscript.  That is what
makes the agent-level surrogate exactly convex rather than merely linearisable.

Two oracles are provided:

``RangeBearingOracle``
    Canonical planar range-bearing sensor.  Radial radius is constant
    (``sigma_r``) and the tangential radius grows linearly with range
    (``r * sigma_phi``), so the set is elongated *across* the line of sight.

``MonocularPoseOracle``
    Model of a learned 6-DOF pose estimator fed by a monocular camera.
    Depth is the weak direction: the along-line-of-sight radius grows
    quadratically with range while the lateral radius grows linearly, and both
    are inflated by an illumination penalty that is minimised at the
    best-viewing position (sun behind the Ego).  The set is therefore
    elongated *along* the line of sight, which is what makes two Egos at
    orthogonal bearings strongly complementary.

Both are pure functions of the agent position; the planner never needs the
distribution inside the set, only the set itself.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

_EPS = 1e-12


def _unit(v: np.ndarray) -> tuple[np.ndarray, float]:
    n = float(np.linalg.norm(v))
    if n < _EPS:
        raise ValueError("degenerate line of sight: agent is coincident with the target")
    return v / n, n


class SetOracle(ABC):
    """Point-to-set map from agent position to an output uncertainty ellipsoid."""

    n_z: int

    @abstractmethod
    def radii(self, p: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(sigma, V)``: principal semi-axis lengths and axes at ``p``.

        ``V`` is orthonormal with columns ``V[:, j]`` the axis for ``sigma[j]``.
        """

    @abstractmethod
    def radii_cvx(self, p_expr, p_ref: np.ndarray):
        """Return a list of DCP-convex cvxpy expressions for the radii.

        ``p_expr`` is a cvxpy expression for the position; ``p_ref`` is the
        current SCvx reference, used only where a term must be frozen.
        """

    # ---- derived quantities -------------------------------------------------

    def shape(self, p: np.ndarray) -> np.ndarray:
        """Ellipsoid shape matrix ``Q``."""
        sigma, V = self.radii(p)
        return (V * sigma**2) @ V.T

    def information(self, p: np.ndarray) -> np.ndarray:
        """Inverse shape matrix ``Q^{-1}`` (the per-agent information matrix)."""
        sigma, V = self.radii(p)
        return (V / sigma**2) @ V.T

    def size(self, p: np.ndarray) -> float:
        """Sum of principal radii -- the convex surrogate for set volume.

        Proposition (DO-Volume) of the manuscript bounds the degree of
        observability below by a multiple of this quantity, and AM-GM relates
        it to ``Vol^{1/n_z}``.  Unlike the volume itself it is convex in ``p``.
        """
        sigma, _ = self.radii(p)
        return float(np.sum(sigma))

    def volume(self, p: np.ndarray) -> float:
        """Euclidean volume of the uncertainty ellipsoid."""
        from .observability import ellipsoid_volume

        sigma, _ = self.radii(p)
        return ellipsoid_volume(sigma)

    def information_jacobian(self, p: np.ndarray, h: float = 1e-4) -> np.ndarray:
        """Central-difference Jacobian of ``Q^{-1}`` w.r.t. ``p``.

        Returns an array of shape ``(n_p, n_z, n_z)`` whose ``k``-th slice is
        ``d(Q^{-1})/dp_k``.  Used to build the affine matrix expression that
        keeps ``-log det`` DCP inside the SCvx subproblem.
        """
        p = np.asarray(p, dtype=float)
        n_p = p.size
        J = np.zeros((n_p, self.n_z, self.n_z))
        scale = max(1.0, float(np.linalg.norm(p)))
        step = h * scale
        for k in range(n_p):
            e = np.zeros(n_p)
            e[k] = step
            J[k] = (self.information(p + e) - self.information(p - e)) / (2.0 * step)
        return J


@dataclass
class RangeBearingOracle(SetOracle):
    """Planar range-bearing sensor observing a fixed target position.

    Parameters
    ----------
    target:
        Target position in the plane.
    sigma_r:
        Range standard deviation (metres).  Constant, along the line of sight.
    sigma_phi:
        Bearing standard deviation (radians).  Produces a tangential radius
        ``r * sigma_phi``.
    """

    target: np.ndarray
    sigma_r: float = 0.5
    sigma_phi: float = 0.05
    n_z: int = 2

    def __post_init__(self) -> None:
        self.target = np.asarray(self.target, dtype=float)
        if self.target.size != 2:
            raise ValueError("RangeBearingOracle is planar; target must be 2-D")

    def radii(self, p: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        u, r = _unit(self.target - np.asarray(p, dtype=float))
        u_perp = np.array([-u[1], u[0]])
        sigma = np.array([self.sigma_r, r * self.sigma_phi])
        V = np.column_stack([u, u_perp])
        return sigma, V

    def radii_cvx(self, p_expr, p_ref: np.ndarray):
        import cvxpy as cvx

        r = cvx.norm(self.target - p_expr, 2)
        return [cvx.Constant(self.sigma_r), self.sigma_phi * r]

    def anisotropy(self, p: np.ndarray) -> float:
        """Ratio of the larger to the smaller radius; 1 means isotropic."""
        sigma, _ = self.radii(p)
        return float(np.max(sigma) / np.min(sigma))


@dataclass
class MonocularPoseOracle(SetOracle):
    """Learned monocular 6-DOF pose estimator, as a state-dependent ellipsoid.

    The along-line-of-sight (depth) radius and the lateral radius are

    .. math::
        \\sigma_{d}(p) = a_d (r/r_0)^2 + w\\,\\|p - p^\\star\\|^2 / r_0^2, \\qquad
        \\sigma_{l}(p) = a_l (r/r_0)   + w\\,\\|p - p^\\star\\|^2 / r_0^2,

    with ``r = \\|z - p\\|`` the range to the Target and ``p^\\star`` the
    best-illumination viewing position (Ego between the Sun and the Target, so
    the Sun is at the Ego's back).  Both are convex in ``p``: ``r`` is a norm,
    ``r^2`` is a convex increasing function of it, and the illumination term is
    a squared norm.  The quadratic depth growth and linear lateral growth are
    the standard monocular scaling -- depth from apparent size degrades one
    order faster than lateral position from pixel bearing.

    Parameters
    ----------
    target:
        Target position (Hill-frame origin in the satellite study).
    sun_direction:
        Unit vector along which sunlight *propagates*.  The best viewing
        position is at ``-sun_direction`` from the Target.
    r_ref:
        Reference range ``r_0``; also the range at which the best-illumination
        position sits.
    a_depth, a_lateral:
        Radii at the reference range with perfect illumination (metres).
    w_illum:
        Weight of the illumination penalty.  Zero recovers a purely
        range-driven oracle.
    """

    target: np.ndarray
    sun_direction: np.ndarray
    r_ref: float = 20.0
    a_depth: float = 1.2
    a_lateral: float = 0.15
    w_illum: float = 0.35
    n_z: int = 3

    def __post_init__(self) -> None:
        self.target = np.asarray(self.target, dtype=float)
        s = np.asarray(self.sun_direction, dtype=float)
        self.sun_direction = s / np.linalg.norm(s)
        self.n_z = int(self.target.size)
        # Best viewing position: sun behind the Ego, at the reference range.
        self.p_star = self.target - self.r_ref * self.sun_direction

    # ---- radii --------------------------------------------------------------

    def _illumination_penalty(self, p: np.ndarray) -> float:
        d = np.asarray(p, dtype=float) - self.p_star
        return self.w_illum * float(d @ d) / self.r_ref**2

    def radii(self, p: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        p = np.asarray(p, dtype=float)
        u, r = _unit(self.target - p)
        pen = self._illumination_penalty(p)
        sigma_d = self.a_depth * (r / self.r_ref) ** 2 + pen
        sigma_l = self.a_lateral * (r / self.r_ref) + pen
        sigma = np.concatenate([[sigma_d], np.full(self.n_z - 1, sigma_l)])
        V = _orthonormal_frame(u)
        return sigma, V

    def radii_cvx(self, p_expr, p_ref: np.ndarray):
        import cvxpy as cvx

        # r**2 written as sum_squares keeps the expression quadratic rather
        # than square-of-norm, which is friendlier to the DCP analyser.
        r_sq = cvx.sum_squares(self.target - p_expr)
        r = cvx.norm(self.target - p_expr, 2)
        pen = self.w_illum * cvx.sum_squares(p_expr - self.p_star) / self.r_ref**2
        sigma_d = self.a_depth * r_sq / self.r_ref**2 + pen
        sigma_l = self.a_lateral * r / self.r_ref + pen
        return [sigma_d] + [sigma_l] * (self.n_z - 1)

    def phase_angle(self, p: np.ndarray) -> float:
        """Sun phase angle at ``p`` (radians); 0 is fully lit, pi is backlit."""
        u, _ = _unit(np.asarray(p, dtype=float) - self.target)
        return float(np.arccos(np.clip(-u @ self.sun_direction, -1.0, 1.0)))

    def anisotropy(self, p: np.ndarray) -> float:
        sigma, _ = self.radii(p)
        return float(np.max(sigma) / np.min(sigma))


def _orthonormal_frame(u: np.ndarray) -> np.ndarray:
    """Orthonormal basis whose first column is ``u``."""
    n = u.size
    if n == 2:
        return np.column_stack([u, np.array([-u[1], u[0]])])
    # Householder-free Gram-Schmidt against the least-aligned coordinate axis.
    seed = np.eye(n)[int(np.argmin(np.abs(u)))]
    v1 = seed - (seed @ u) * u
    v1 /= np.linalg.norm(v1)
    cols = [u, v1]
    if n == 3:
        cols.append(np.cross(u, v1))
    else:  # pragma: no cover - only 2-D and 3-D are used in the studies
        for axis in np.eye(n):
            w = axis - sum((axis @ c) * c for c in cols)
            if np.linalg.norm(w) > 1e-8:
                cols.append(w / np.linalg.norm(w))
            if len(cols) == n:
                break
    return np.column_stack(cols)
