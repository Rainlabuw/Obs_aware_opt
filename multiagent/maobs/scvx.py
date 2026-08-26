"""Alternating successive convexification for estimation-aware multi-agent planning.

This implements Algorithm 1 of the manuscript: a Gauss-Seidel sweep over agents
in which each agent solves one trust-region-managed convex subproblem
(Problem 2) with its neighbours' trajectories -- and hence their information
matrices -- frozen.

Structure of the subproblem
---------------------------
The agent dynamics are LTI, so the dynamics constraint is *exact* after the
change of variables ``x = xbar + dx``; no virtual control or defect penalty is
needed (unlike the single-agent nonlinear case in ``sat_rendezvous``).  The
remaining non-convexities are handled as follows.

``own set size``
    ``Lambda_i(p) = sum_j sigma_j(p)`` is convex in ``p`` by construction of
    the oracle, so it enters the subproblem *exactly* -- it is never
    linearised.  This is the practical payoff of the directional-convexity
    assumption.

``network term``
    ``-log det (S_{-i} + Q_i(p)^{-1})``.  The information matrix
    ``M_i(p) = Q_i(p)^{-1}`` is replaced by its first-order expansion in ``dp``,
    which is an *affine matrix* expression; ``-log det`` of an affine PSD
    argument is convex, so the curvature of the log-determinant is retained
    rather than being flattened by a scalar linearisation.

``keep-out and inter-agent separation``
    Non-convex distance-lower-bound constraints, linearised about the
    reference and softened with L1-penalised slacks.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cvxpy as cvx
import numpy as np

from .dynamics import LinearAgent
from .observability import fuse, volume_from_shape


@dataclass
class PlannerConfig:
    """Weights, constraints and trust-region parameters for Algorithm 1."""

    horizon: int = 60

    # primary task
    w_state: float = 1.0
    w_control: float = 1.0
    w_terminal: float = 20.0
    Q_state: np.ndarray | None = None
    R_control: np.ndarray | None = None

    # estimation-aware terms
    alpha_obs: float = 1.0
    lambda_net: float = 1.0

    # constraints
    d_keepout: float = 10.0
    d_collision: float = 4.0
    u_max: float = 0.05
    penalty: float = 1e3

    # trust region / SCvx
    trust_radius: float = 5.0
    trust_min: float = 1e-3
    trust_max: float = 50.0
    shrink: float = 2.0
    grow: float = 1.6
    rho0: float = 0.0
    rho1: float = 0.25
    rho2: float = 0.7

    # outer loop
    max_outer: int = 25
    tol: float = 1e-3  # on max |dx| across agents
    obj_tol: float = 1e-4  # on relative change of the joint objective

    free_terminal: bool = True
    solver: str = "CLARABEL"
    verbose: bool = False


@dataclass
class PlannerResult:
    x: np.ndarray  # (m, nx, T+1)
    u: np.ndarray  # (m, nu, T)
    objective_history: list[float] = field(default_factory=list)
    per_agent_objective_history: list[list[float]] = field(default_factory=list)
    trust_history: list[list[float]] = field(default_factory=list)
    step_history: list[float] = field(default_factory=list)
    outer_iterations: int = 0
    converged: bool = False
    convergence_reason: str = "max_outer"
    solve_time: float = 0.0
    subproblems_solved: int = 0
    inaccurate_solves: int = 0
    failed_solves: int = 0


# ---------------------------------------------------------------------------
# cost pieces
# ---------------------------------------------------------------------------


def _quad(cfg: PlannerConfig, nx: int, nu: int) -> tuple[np.ndarray, np.ndarray]:
    Q = cfg.Q_state if cfg.Q_state is not None else np.eye(nx)
    R = cfg.R_control if cfg.R_control is not None else np.eye(nu)
    return Q, R


def task_cost(cfg: PlannerConfig, agent: LinearAgent, x: np.ndarray, u: np.ndarray,
              x_des: np.ndarray) -> float:
    """Primary (estimation-agnostic) cost of a single agent trajectory."""
    Q, R = _quad(cfg, agent.nx, agent.nu)
    T = u.shape[1]
    J = 0.0
    for t in range(T):
        e = x[:, t] - x_des
        J += cfg.w_state * float(e @ Q @ e) + cfg.w_control * float(u[:, t] @ R @ u[:, t])
    eN = x[:, T] - x_des
    J += cfg.w_terminal * float(eN @ Q @ eN)
    return J


def estimation_cost(cfg: PlannerConfig, agent: LinearAgent, oracle, x: np.ndarray,
                    neighbour_info: np.ndarray | None) -> float:
    """``alpha_obs * sum_t [ Lambda_i(p_t) + lambda_N * (-log det(S_t + M_i(p_t))) ]``."""
    T1 = x.shape[1]
    total = 0.0
    for t in range(T1):
        p = agent.position(x[:, t])
        total += oracle.size(p)
        if neighbour_info is not None and cfg.lambda_net > 0.0:
            M = oracle.information(p)
            _, logdet = np.linalg.slogdet(neighbour_info[t] + M)
            total += cfg.lambda_net * (-logdet)
    return cfg.alpha_obs * total


def constraint_violation(cfg: PlannerConfig, agent: LinearAgent, x: np.ndarray,
                         target: np.ndarray, neighbour_pos: np.ndarray | None) -> float:
    """L1 violation of the non-convex separation constraints."""
    T1 = x.shape[1]
    v = 0.0
    for t in range(T1):
        p = agent.position(x[:, t])
        v += max(0.0, cfg.d_keepout**2 - float((p - target) @ (p - target)))
        if neighbour_pos is not None:
            for j in range(neighbour_pos.shape[0]):
                d = p - neighbour_pos[j, t]
                v += max(0.0, cfg.d_collision**2 - float(d @ d))
    return v


def agent_objective(cfg, agent, oracle, x, u, x_des, target, neighbour_info,
                    neighbour_pos) -> float:
    """Full non-convex agent objective used for the trust-region ratio test."""
    return (
        task_cost(cfg, agent, x, u, x_des)
        + estimation_cost(cfg, agent, oracle, x, neighbour_info)
        + cfg.penalty * constraint_violation(cfg, agent, x, target, neighbour_pos)
    )


# ---------------------------------------------------------------------------
# convex subproblem (Problem 2)
# ---------------------------------------------------------------------------


def solve_subproblem(cfg, agent, oracle, x_ref, u_ref, x_des, target,
                     neighbour_info, neighbour_pos, trust_radius):
    """Solve Problem 2 for one agent.  Returns ``(dx, du, predicted_objective)``."""
    nx, nu = agent.nx, agent.nu
    T = u_ref.shape[1]
    Q, R = _quad(cfg, nx, nu)
    C = agent.C

    dx = cvx.Variable((nx, T + 1), name="dx")
    du = cvx.Variable((nu, T), name="du")
    slack_keep = cvx.Variable(T + 1, nonneg=True)
    n_nb = 0 if neighbour_pos is None else neighbour_pos.shape[0]
    slack_coll = cvx.Variable((n_nb, T + 1), nonneg=True) if n_nb else None

    cons = [dx[:, 0] == 0]
    for t in range(T):
        cons.append(dx[:, t + 1] == agent.Ad @ dx[:, t] + agent.Bd @ du[:, t])
        cons.append(cvx.norm(u_ref[:, t] + du[:, t], 2) <= cfg.u_max)
    cons.append(cvx.norm(dx, "inf") <= trust_radius)
    if not cfg.free_terminal:
        cons.append(dx[:, T] == 0)

    obj = 0
    for t in range(T + 1):
        p_ref = agent.position(x_ref[:, t])
        p = C @ (x_ref[:, t] + dx[:, t])
        dp = C @ dx[:, t]

        # --- primary task ---
        e = x_ref[:, t] + dx[:, t] - x_des
        w = cfg.w_terminal if t == T else cfg.w_state
        obj += w * cvx.quad_form(e, cvx.psd_wrap(Q))
        if t < T:
            obj += cfg.w_control * cvx.quad_form(u_ref[:, t] + du[:, t], cvx.psd_wrap(R))

        # --- own set size: exactly convex, no linearisation ---
        if cfg.alpha_obs > 0.0:
            obj += cfg.alpha_obs * cvx.sum(cvx.hstack(oracle.radii_cvx(p, p_ref)))

            # --- network term: affine information matrix inside -log det ---
            if cfg.lambda_net > 0.0 and neighbour_info is not None:
                M0 = oracle.information(p_ref)
                J = oracle.information_jacobian(p_ref)
                M_expr = cvx.Constant(0.5 * (M0 + M0.T) + neighbour_info[t])
                for k in range(J.shape[0]):
                    Jk = 0.5 * (J[k] + J[k].T)
                    M_expr = M_expr + dp[k] * Jk
                obj += cfg.alpha_obs * cfg.lambda_net * (-cvx.log_det(M_expr))

        # --- linearised keep-out:  d_min^2 - ||p - z||^2 <= slack ---
        g = cfg.d_keepout**2 - float((p_ref - target) @ (p_ref - target))
        grad = -2.0 * (p_ref - target)
        cons.append(g + grad @ dp <= slack_keep[t])
        obj += cfg.penalty * slack_keep[t]

        # --- linearised inter-agent separation ---
        for j in range(n_nb):
            dvec = p_ref - neighbour_pos[j, t]
            gc = cfg.d_collision**2 - float(dvec @ dvec)
            cons.append(gc - 2.0 * dvec @ dp <= slack_coll[j, t])
            obj += cfg.penalty * slack_coll[j, t]

    prob = cvx.Problem(cvx.Minimize(obj), cons)
    try:
        import warnings

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            prob.solve(solver=getattr(cvx, cfg.solver), verbose=cfg.verbose)
    except cvx.error.SolverError:
        return None, None, np.inf, "solver_error"
    if dx.value is None or du.value is None:
        return None, None, np.inf, str(prob.status)
    return dx.value, du.value, float(prob.value), str(prob.status)


# ---------------------------------------------------------------------------
# Algorithm 1
# ---------------------------------------------------------------------------


def plan(agents, oracles, x0, x_des, target, cfg: PlannerConfig,
         u_init=None) -> PlannerResult:
    """Alternating estimation-aware planning (Algorithm 1).

    Parameters
    ----------
    agents:
        list of ``LinearAgent`` (one per Ego; may be the same object repeated).
    oracles:
        list of ``SetOracle``, one per agent.
    x0, x_des:
        arrays ``(m, nx)`` of initial and desired states.
    target:
        network state (Target position) in ``R^{n_z}``.
    u_init:
        optional ``(m, nu, T)`` warm start; defaults to the LQR-free zero input.
    """
    import time

    m = len(agents)
    T = cfg.horizon
    nx, nu = agents[0].nx, agents[0].nu
    x0 = np.asarray(x0, dtype=float)
    x_des = np.asarray(x_des, dtype=float)
    target = np.asarray(target, dtype=float)

    u = np.zeros((m, nu, T)) if u_init is None else np.array(u_init, dtype=float)
    x = np.stack([agents[i].rollout(x0[i], u[i]) for i in range(m)])

    trust = [cfg.trust_radius] * m
    res = PlannerResult(x=x.copy(), u=u.copy())
    t_start = time.time()

    def positions(idx_exclude=None):
        out = []
        for i in range(m):
            if i == idx_exclude:
                continue
            out.append(np.stack([agents[i].position(x[i][:, t]) for t in range(T + 1)]))
        return np.stack(out) if out else None

    def neighbour_information(idx):
        if m == 1 or cfg.lambda_net <= 0.0:
            return None
        S = np.zeros((T + 1, oracles[idx].n_z, oracles[idx].n_z))
        for t in range(T + 1):
            for j in range(m):
                if j == idx:
                    continue
                S[t] += oracles[j].information(agents[j].position(x[j][:, t]))
        return S

    for outer in range(cfg.max_outer):
        step_norm = 0.0
        per_agent_obj = []
        for i in range(m):
            S = neighbour_information(i)
            P = positions(idx_exclude=i)
            J_before = agent_objective(cfg, agents[i], oracles[i], x[i], u[i],
                                       x_des[i], target, S, P)

            dx, du, J_lin, status = solve_subproblem(cfg, agents[i], oracles[i], x[i], u[i],
                                                     x_des[i], target, S, P, trust[i])
            res.subproblems_solved += 1
            if "inaccurate" in status:
                res.inaccurate_solves += 1
            if dx is None:
                res.failed_solves += 1
                trust[i] = max(cfg.trust_min, trust[i] / cfg.shrink)
                per_agent_obj.append(J_before)
                continue

            x_try = agents[i].rollout(x0[i], u[i] + du)
            J_after = agent_objective(cfg, agents[i], oracles[i], x_try, u[i] + du,
                                      x_des[i], target, S, P)

            dJ = J_before - J_after
            dL = J_before - J_lin
            rho = dJ / dL if abs(dL) > 1e-12 else (1.0 if dJ > 0 else -1.0)

            if rho < cfg.rho0:
                trust[i] = max(cfg.trust_min, trust[i] / cfg.shrink)
                per_agent_obj.append(J_before)
                continue

            step_norm = max(step_norm, float(np.max(np.abs(x_try - x[i]))))
            x[i], u[i] = x_try, u[i] + du
            per_agent_obj.append(J_after)

            if rho < cfg.rho1:
                trust[i] = max(cfg.trust_min, trust[i] / cfg.shrink)
            elif rho >= cfg.rho2:
                trust[i] = min(cfg.trust_max, trust[i] * cfg.grow)

        total = _network_objective(cfg, agents, oracles, x, u, x_des, target)
        res.objective_history.append(total)
        res.per_agent_objective_history.append(per_agent_obj)
        res.trust_history.append(list(trust))
        res.step_history.append(step_norm)
        res.outer_iterations = outer + 1

        if step_norm < cfg.tol:
            res.converged = True
            res.convergence_reason = "step"
            break
        if len(res.objective_history) >= 2:
            prev = res.objective_history[-2]
            rel = abs(total - prev) / max(abs(prev), 1e-12)
            if rel < cfg.obj_tol:
                res.converged = True
                res.convergence_reason = "objective"
                break

    res.x, res.u = x, u
    res.solve_time = time.time() - t_start
    return res


def _network_objective(cfg, agents, oracles, x, u, x_des, target) -> float:
    """Joint objective: task cost of every agent plus the fused-set penalty.

    This is the quantity the alternating scheme is (heuristically) descending
    on; it is reported per outer iteration for the convergence study.
    """
    m = len(agents)
    T = u.shape[2]
    J = sum(task_cost(cfg, agents[i], x[i], u[i], x_des[i]) for i in range(m))
    for t in range(T + 1):
        infos = [oracles[i].information(agents[i].position(x[i][:, t])) for i in range(m)]
        S = np.sum(np.stack(infos), axis=0)
        _, logdet = np.linalg.slogdet(S)
        J += cfg.alpha_obs * cfg.lambda_net * (-logdet)
        J += cfg.alpha_obs * sum(
            oracles[i].size(agents[i].position(x[i][:, t])) for i in range(m)
        )
    return float(J)
