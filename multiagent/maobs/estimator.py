"""Estimators driven by set-valued (non-Gaussian) measurements.

The premise of the framework is that the measurement error is *bounded* but of
unknown distribution.  We therefore drive both estimators with errors sampled
uniformly from the state-dependent uncertainty ellipsoid -- a distribution the
Gaussian filter does not know and cannot represent -- and compare:

``GaussianFusionFilter``
    What a practitioner actually deploys: a linear-Gaussian fusion filter given
    the covariance-matched noise ``R = Q/(n+2)`` (the covariance of the uniform
    distribution on ``E(0, Q)``).  Reports RMSE.  This is a *mismatched* filter
    by construction, and that is the point -- the planning gain should survive
    the mismatch.

``SetMembershipFilter``
    The estimator the theory actually matches: intersect the measurement
    ellipsoids.  Because ``z`` lies in every measurement ellipsoid by
    construction, the running intersection is a *guaranteed* containment set,
    so its radius is a hard worst-case error bound rather than a statistical
    one.  Its volume is precisely the ``Vol(Y_z)`` the planner minimises.

Ellipsoid intersection uses the classical parametrised outer bound
(Fogel & Huang; Durieu, Walter & Polyak), minimised over the parameter.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .observability import volume_from_shape


def sample_in_ellipsoid(sigma: np.ndarray, V: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """Draw one sample uniformly from ``E(0, V diag(sigma^2) V^T)``."""
    n = sigma.size
    d = rng.normal(size=n)
    d /= np.linalg.norm(d)
    radius = rng.random() ** (1.0 / n)
    return V @ (sigma * d * radius)


def measurement(oracle, p: np.ndarray, truth: np.ndarray,
                rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    """One agent measurement of the Target and the shape matrix that bounds it."""
    sigma, V = oracle.radii(p)
    y = truth + sample_in_ellipsoid(sigma, V, rng)
    Q = (V * sigma**2) @ V.T
    return y, Q


# ---------------------------------------------------------------------------
# Gaussian fusion (mismatched)
# ---------------------------------------------------------------------------


class GaussianFusionFilter:
    """Information-form fusion filter for a static network state."""

    def __init__(self, n: int, prior_scale: float = 1e4):
        self.n = n
        self.info = np.eye(n) / prior_scale**2
        self.info_state = np.zeros(n)

    def update(self, y: np.ndarray, Q: np.ndarray) -> None:
        # Covariance of a uniform draw on E(0, Q) is Q/(n+2).
        R_inv = np.linalg.inv(Q / (self.n + 2))
        self.info = self.info + R_inv
        self.info_state = self.info_state + R_inv @ y

    @property
    def estimate(self) -> np.ndarray:
        return np.linalg.solve(self.info, self.info_state)

    @property
    def covariance(self) -> np.ndarray:
        return np.linalg.inv(self.info)


# ---------------------------------------------------------------------------
# Set membership (matched)
# ---------------------------------------------------------------------------


def intersect_ellipsoids(c1, Q1, c2, Q2, n_grid: int = 41):
    """Tight-ish outer ellipsoid of ``E(c1,Q1) ∩ E(c2,Q2)``.

    Parametrised family: for ``lam in [0,1]``,
    ``Q_lam = (lam Q1^-1 + (1-lam) Q2^-1)^-1``,
    ``c_lam = Q_lam (lam Q1^-1 c1 + (1-lam) Q2^-1 c2)``,
    ``k_lam = 1 - lam(1-lam) (c2-c1)^T (lam Q2 + (1-lam) Q1)^-1 (c2-c1)``,
    and ``E(c_lam, k_lam Q_lam)`` contains the intersection.  We grid-search
    ``lam`` for the smallest determinant.
    """
    P1, P2 = np.linalg.inv(Q1), np.linalg.inv(Q2)
    dc = c2 - c1
    best = None
    for lam in np.linspace(0.0, 1.0, n_grid):
        S = lam * P1 + (1.0 - lam) * P2
        try:
            Ql = np.linalg.inv(S)
        except np.linalg.LinAlgError:  # pragma: no cover
            continue
        cl = Ql @ (lam * P1 @ c1 + (1.0 - lam) * P2 @ c2)
        M = lam * Q2 + (1.0 - lam) * Q1
        k = 1.0 - lam * (1.0 - lam) * float(dc @ np.linalg.solve(M, dc))
        if k <= 0:
            continue  # empty intersection under this bound
        Qk = k * Ql
        sign, logdet = np.linalg.slogdet(Qk)
        if sign <= 0:
            continue
        if best is None or logdet < best[0]:
            best = (logdet, cl, Qk)
    if best is None:  # pragma: no cover - degenerate, fall back to the smaller set
        return (c1, Q1) if np.linalg.det(Q1) < np.linalg.det(Q2) else (c2, Q2)
    return best[1], best[2]


class SetMembershipFilter:
    """Running intersection of measurement ellipsoids for a static state."""

    def __init__(self, n: int, prior_scale: float = 1e3):
        self.n = n
        self.center = np.zeros(n)
        self.shape = np.eye(n) * prior_scale**2
        self._initialised = False

    def update(self, y: np.ndarray, Q: np.ndarray) -> None:
        if not self._initialised:
            self.center, self.shape = y.copy(), Q.copy()
            self._initialised = True
            return
        self.center, self.shape = intersect_ellipsoids(self.center, self.shape, y, Q)

    @property
    def estimate(self) -> np.ndarray:
        return self.center

    @property
    def worst_case_radius(self) -> float:
        """Largest semi-axis: a hard bound on ``||zhat - z||``."""
        return float(np.sqrt(max(np.linalg.eigvalsh(self.shape)[-1], 0.0)))

    @property
    def volume(self) -> float:
        return volume_from_shape(self.shape)


# ---------------------------------------------------------------------------
# Monte Carlo driver
# ---------------------------------------------------------------------------


@dataclass
class EstimationStats:
    rmse: np.ndarray  # (T+1,) Gaussian-filter RMSE over trials
    mean_error: np.ndarray
    max_error: np.ndarray
    sm_radius: np.ndarray  # (T+1,) set-membership guaranteed radius (mean over trials)
    sm_volume: np.ndarray
    sm_error: np.ndarray  # set-membership centre error, RMSE over trials
    coverage: float  # fraction of steps where the truth was inside the SM set
    final_rmse: float = 0.0
    final_sm_radius: float = 0.0

    def __post_init__(self) -> None:
        self.final_rmse = float(self.rmse[-1])
        self.final_sm_radius = float(self.sm_radius[-1])


def monte_carlo(oracles, positions: np.ndarray, truth: np.ndarray,
                trials: int = 500, seed: int = 0,
                run_set_membership: bool = True) -> EstimationStats:
    """Monte Carlo estimation study along a fixed set of agent trajectories.

    ``positions`` has shape ``(m, T+1, n_p)``; ``truth`` is the Target state in
    ``R^{n_z}``.  Trajectories are held fixed -- the planner has already run --
    so this isolates estimation quality attributable to geometry.
    """
    m, T1, _ = positions.shape
    n = truth.size
    rng = np.random.default_rng(seed)

    err = np.zeros((trials, T1))
    sm_err = np.zeros((trials, T1))
    sm_rad = np.zeros((trials, T1))
    sm_vol = np.zeros((trials, T1))
    inside = 0
    checks = 0

    for k in range(trials):
        gf = GaussianFusionFilter(n)
        sm = SetMembershipFilter(n) if run_set_membership else None
        for t in range(T1):
            for i in range(m):
                y, Q = measurement(oracles[i], positions[i, t], truth, rng)
                gf.update(y, Q)
                if sm is not None:
                    sm.update(y, Q)
            err[k, t] = np.linalg.norm(gf.estimate - truth)
            if sm is not None:
                sm_err[k, t] = np.linalg.norm(sm.estimate - truth)
                sm_rad[k, t] = sm.worst_case_radius
                sm_vol[k, t] = sm.volume
                d = truth - sm.center
                inside += int(d @ np.linalg.solve(sm.shape, d) <= 1.0 + 1e-9)
                checks += 1

    return EstimationStats(
        rmse=np.sqrt(np.mean(err**2, axis=0)),
        mean_error=np.mean(err, axis=0),
        max_error=np.max(err, axis=0),
        sm_radius=np.mean(sm_rad, axis=0),
        sm_volume=np.mean(sm_vol, axis=0),
        sm_error=np.sqrt(np.mean(sm_err**2, axis=0)),
        coverage=float(inside / checks) if checks else float("nan"),
    )
