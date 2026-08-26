"""Network observability metrics for set-valued output uncertainties.

Everything in this module is *evaluation only* -- these are the quantities the
paper reports, not the surrogates the planner optimises.  Keeping them separate
matters: the surrogate is convex by construction, the metric is not, and the
whole point of the experiments is to check that improving the surrogate really
does improve the metric.

Conventions follow the manuscript: an uncertainty set is the ellipsoid
``E(z, Q) = {z' : (z'-z)^T Q^{-1} (z'-z) <= 1}``, so the support radius of
``E(0, Q)`` in a unit direction ``nu`` is ``sqrt(nu^T Q nu)``.
"""

from __future__ import annotations

import numpy as np
from scipy.special import gammaln


def ellipsoid_volume(sigma: np.ndarray) -> float:
    """Volume of an ellipsoid with semi-axis lengths ``sigma``."""
    sigma = np.asarray(sigma, dtype=float)
    n = sigma.size
    log_unit_ball = (n / 2.0) * np.log(np.pi) - gammaln(n / 2.0 + 1.0)
    return float(np.exp(log_unit_ball + np.sum(np.log(sigma))))


def volume_from_shape(Q: np.ndarray) -> float:
    """Volume of ``E(., Q)`` from its shape matrix."""
    sign, logdet = np.linalg.slogdet(Q)
    if sign <= 0:
        return 0.0
    n = Q.shape[0]
    log_unit_ball = (n / 2.0) * np.log(np.pi) - gammaln(n / 2.0 + 1.0)
    return float(np.exp(log_unit_ball + 0.5 * logdet))


def support_radius(Q: np.ndarray, nu: np.ndarray) -> float:
    """Radius of ``E(0, Q)`` along the unit direction ``nu``."""
    return float(np.sqrt(max(nu @ Q @ nu, 0.0)))


def fuse(information_matrices: list[np.ndarray]) -> np.ndarray:
    """Shape matrix of the fused (intersected) network uncertainty set.

    Uses the information-matrix approximation of the ellipsoid intersection,
    ``Q_z = (sum_i Q_i^{-1})^{-1}``, exact for co-centred ellipsoids and the
    standard outer approximation otherwise.  This is the ``Y_z = \\cap_i Y_z^i``
    of the manuscript.
    """
    S = np.sum(np.stack(information_matrices, axis=0), axis=0)
    return np.linalg.inv(S)


def is_informative(
    agent_shapes: list[np.ndarray], fused_shape: np.ndarray, rtol: float = 1e-9
) -> bool:
    """Network Informativity test: the intersection beats every individual set."""
    v_net = volume_from_shape(fused_shape)
    if v_net <= 0.0:
        return False
    return all(v_net < volume_from_shape(Q) * (1.0 - rtol) for Q in agent_shapes)


def informativity_ratio(agent_shapes: list[np.ndarray], fused_shape: np.ndarray) -> float:
    """``Vol(Y_z) / min_i Vol(Y_z^i)``.  Below one means the network is informative."""
    v_net = volume_from_shape(fused_shape)
    v_min = min(volume_from_shape(Q) for Q in agent_shapes)
    return float(v_net / v_min) if v_min > 0 else np.inf


def total_directional_radius(fused_shapes: list[np.ndarray]) -> float:
    """``sum_t sum_i sqrt(nu_i^T Q_{z,t} nu_i)`` over the coordinate directions.

    This is the quantity the degree of observability trades against: see
    :func:`degree_of_observability`.  It is independent of the probe size
    ``epsilon`` and so is the more convenient number to report and to compare
    across scenarios.
    """
    total = 0.0
    for Q in fused_shapes:
        total += float(np.sum(np.sqrt(np.clip(np.diag(Q), 0.0, None))))
    return total


def degree_of_observability(fused_shapes: list[np.ndarray], epsilon: float) -> float:
    """Degree of observability of the network output tube.

    Perturbing the network state by ``+/- epsilon`` along each coordinate
    direction ``nu_i`` and propagating gives two tubes whose sets at time ``t``
    are centred ``2*epsilon`` apart along ``nu_i``.  Their set distance is

    ``d = max(0, 2*epsilon - 2*sqrt(nu_i^T Q_{z,t} nu_i))``,

    since the support radius of each set along ``nu_i`` is
    ``sqrt(nu_i^T Q nu_i)``.  Summing over directions and time gives the
    manuscript's ``D_O``.  The clamp at zero encodes that the pseudo-metric
    reports zero for non-separated sets.

    Note the immediate consequence, which is worth stating in the paper: for a
    static network state ``D_O`` is an affine, strictly decreasing function of
    :func:`total_directional_radius` up to the clamp -- maximising ``D_O`` is
    *exactly* minimising the summed directional radii, no bounding argument
    required.
    """
    total = 0.0
    for Q in fused_shapes:
        radii = np.sqrt(np.clip(np.diag(Q), 0.0, None))
        total += float(np.sum(np.maximum(0.0, 2.0 * (epsilon - radii))))
    return total


def network_shapes(
    oracles: list, positions: np.ndarray
) -> tuple[list[np.ndarray], list[list[np.ndarray]]]:
    """Fused shape matrices along a trajectory.

    ``positions`` has shape ``(m, T+1, n_p)``.  Returns the fused shape at each
    time step and, per time step, the list of individual agent shapes.
    """
    m, T1, _ = positions.shape
    fused, per_agent = [], []
    for t in range(T1):
        infos = [oracles[i].information(positions[i, t]) for i in range(m)]
        shapes = [np.linalg.inv(M) for M in infos]
        fused.append(np.linalg.inv(np.sum(np.stack(infos, axis=0), axis=0)))
        per_agent.append(shapes)
    return fused, per_agent


def degree_of_network_observability(
    oracle, position: np.ndarray, neighbour_information: np.ndarray
) -> float:
    """``D_N`` for one agent at one time step.

    The manuscript defines ``D_N`` as the tube distance between the agent's
    actual output tube and the most informative tube it could present given
    that its neighbours are held fixed.  With the information-matrix fusion
    model the most informative configuration maximises
    ``log det (S_{-i} + Q_i^{-1})``, so the natural realisation of that
    distance is the log-determinant gap

    ``D_N = log det(S_{-i} + Q_i^{star -1}) - log det(S_{-i} + Q_i^{-1}) >= 0``,

    which is zero exactly at the optimal configuration.  Here the optimum is
    approximated by the best achievable orientation at the *same* set radii,
    i.e. the agent keeps its uncertainty magnitudes and only its geometry is
    scored.  That isolates the network contribution from the single-agent one.
    """
    sigma, _ = oracle.radii(position)
    M_i = oracle.information(position)
    S = neighbour_information

    best = -np.inf
    for V in _candidate_frames(S, sigma.size):
        M_try = (V / sigma**2) @ V.T
        _, logdet = np.linalg.slogdet(S + M_try)
        best = max(best, logdet)
    _, logdet_actual = np.linalg.slogdet(S + M_i)
    return float(max(0.0, best - logdet_actual))


def _candidate_frames(S: np.ndarray, n: int) -> list[np.ndarray]:
    """Eigenframe of the neighbour information and its axis permutations.

    The best orientation for the agent's own set puts its *smallest* radius
    (largest information) along the direction in which the neighbours are
    weakest, i.e. aligns the agent frame with the eigenvectors of ``S``.  We
    enumerate the axis permutations of that eigenframe, which for ``n <= 3``
    is cheap and contains the optimum.
    """
    from itertools import permutations

    _, V = np.linalg.eigh(S)
    return [V[:, list(perm)] for perm in permutations(range(n))]
