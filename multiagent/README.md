# Estimation-aware multi-agent planning with set-valued uncertainty

Reference implementation for the multi-agent extension of
[Deole & Mesbahi, *Estimation-Aware Trajectory Optimization with Set-Valued
Measurement Uncertainties*](https://arxiv.org/abs/2501.09192).

Where the single-agent work (`../sat_rendezvous`, `../dubincar`) shrinks *one*
agent's uncertainty set, this package asks a different question: given several
sensing agents whose measurement uncertainty is state-dependent and
**anisotropic**, where should they go so that the *intersection* of their
uncertainty sets is small? That is the network observability problem.

The headline empirical finding is that these are not the same objective. A
planner in which every agent independently minimises its own set size performs
almost identically to an estimation-agnostic planner; the gain comes
specifically from the network coupling term.

## Layout

```
maobs/                  library — no plotting, returns arrays only
  dynamics.py           planar double integrator; Clohessy-Wiltshire
  oracle.py             state-dependent set-valued output oracles
  observability.py      D_O, fused sets, informativity, degree of network obs.
  scvx.py               alternating successive convexification (Algorithm 1)
  estimator.py          Gaussian fusion filter + set-membership filter, Monte Carlo
experiments/
  scenario_satellite.py scenario constants for the LEO study
  run_planar.py         planar two-robot study + analytic sweep
  run_satellite.py      multi-Ego satellite study, ablation, convergence
  common.py             metric extraction and result I/O
  figures.py            the ONLY place matplotlib is imported
tests/test_core.py      sanity checks incl. the analytic orthogonality identity
results/                .json summaries and .npz arrays
figures/                .png output
```

The separation is deliberate: `maobs` and every `run_*.py` are
visualisation-free, so the simulation can run headless and the figures are
regenerated from stored arrays.

## Install and run

```bash
python -m venv .venv && .venv/Scripts/activate
pip install -r requirements.txt
```

```bash
python tests/test_core.py
```

```bash
cd experiments
python run_planar.py
python run_satellite.py --m 3 --trials 400
python figures.py
```

## Model

**Uncertainty sets.** Each agent's output uncertainty is the ellipsoid
`E(z, Q(x)) = {z' : (z'-z)^T Q(x)^{-1} (z'-z) <= 1}`, with `Q = V diag(sigma^2) V^T`.
Every oracle is written so each principal radius `sigma_j` is a **convex**
function of agent position — the manuscript's directional-convexity
assumption. That is what makes the agent-level surrogate exactly convex
instead of merely linearisable.

- `RangeBearingOracle` — planar, radii `(sigma_r, r sigma_phi)`: elongated
  *across* the line of sight.
- `MonocularPoseOracle` — models a learned 6-DOF pose head. Depth is the weak
  direction: the along-line-of-sight radius grows quadratically with range,
  the lateral radius linearly, and both are inflated away from the
  best-illumination viewing position. Elongated *along* the line of sight,
  which is what makes two Egos at orthogonal bearings complementary.

**Network set.** `Q_z = (sum_i Q_i^{-1})^{-1}` — the information-matrix
approximation of the ellipsoid intersection, exact for co-centred sets.

**Degree of observability.** Because the support radius of `E(0,Q)` along a
unit direction `nu` is `sqrt(nu^T Q nu)`, probing the network state by
`+/- epsilon` along each coordinate direction gives, whenever `epsilon`
exceeds the fused radius,

```
D_O = 2 [ n_z (T+1) epsilon  -  sum_t sum_i sqrt( nu_i^T Q_{z,t} nu_i ) ]
```

*exactly*. Maximising `D_O` is identically minimising the summed directional
radii of the fused tube — no Lipschitz or AM-GM bounding chain, and no
undetermined constants. Verified to machine precision in `tests/test_core.py`.

**Planner.** Gauss-Seidel sweep over agents; each solves one trust-region
convex subproblem with neighbours frozen. The agent's own set size enters
*exactly* (it is convex); only the network term
`-log det(S_{-i} + Q_i^{-1})` is linearised, and it keeps its curvature by
putting an affine-in-`dp` information matrix inside `-log det`, which is
DCP-convex. Agent dynamics are LTI, so the dynamics constraint is exact and no
virtual control is needed.

**Estimators.** Measurement errors are drawn *uniformly from the ellipsoid* —
a distribution the filter does not know:

- `GaussianFusionFilter` — covariance-matched (`R = Q/(n+2)`) but structurally
  mismatched. The practical baseline; reports RMSE.
- `SetMembershipFilter` — intersects the measurement ellipsoids. Because the
  truth lies in every one by construction, this is a **guaranteed** containment
  set, so its radius is a hard worst-case bound. Coverage is checked and comes
  out at 1.0.

## Analytic result reproduced by the planner

For two co-centred planar ellipses with radii `(a,b)` at bearings
`phi_1, phi_2`, with `alpha = 1/a^2`, `beta = 1/b^2`:

```
det(Q_1^{-1} + Q_2^{-1}) = (alpha+beta)^2 - (alpha-beta)^2 cos^2(phi_1 - phi_2)
```

maximised at `|dphi| = pi/2`, with fused-volume gain over the parallel
configuration exactly `(alpha+beta) / (2 sqrt(alpha beta))`. The gain depends
only on the anisotropy ratio and **vanishes for isotropic sets** — the precise
reason non-homogeneous sensing is necessary for the network term to buy
anything. The planner converges to 89.85 degrees.

## Extending the oracle

`SetOracle` needs `radii(p) -> (sigma, V)` and `radii_cvx(p_expr, p_ref)`
returning DCP-convex cvxpy expressions for the same radii. Anything satisfying
that plugs straight into the planner. A learned oracle fitted from Monte Carlo
sweeps of a real pose estimator over range, viewing angle and illumination is
the intended path for the photorealistic pipeline.
