# Follow-on paper: photorealistic oracle from Unreal Engine + learned pose estimation

Decisions and findings as of 2026-08-02. This is the plan for the *next* paper,
not the current submission — the current paper ships with the analytic
`MonocularPoseOracle`.

## Scope decision

The UE + CNN oracle is a **separate follow-on paper**. The current multi-agent
paper is complete on the analytic oracle and its results are strong; gating
submission on the whole perception pipeline would be the wrong risk.

The follow-on paper's contribution is the *measured, certified* oracle itself —
specifically, that a moment-fit Gaussian ellipsoid **under-covers** under real
non-Gaussian estimator error while a conformal set does not, and what that
difference does to the planner.

## Perception stack

```
UE render ──► keypoint net ──► heatmaps ──► conformal ──► keypoint SETS
                                                              │
                              3D landmark model ──► PnP geometry
                                                              ▼
                                                    pose set in SE(3)   (PURSE)
                                                              │
                                                     SLUE ────┤
                                                              ▼
                                              min-volume ellipsoid  =  Q(x)
```

Chosen because it is the only family whose **native output is a bounded,
non-Gaussian set** — which is exactly what the theory assumes. Everything else
(direct regression, MC dropout, ensembles, render-and-compare) yields either a
point estimate or a Gaussian covariance, and turning a covariance into a set
means picking an arbitrary `k`-sigma ellipsoid, which reintroduces the very
assumption the framework rejects. Lipschitz-bounded networks give a genuine hard
bound but one far too loose to plan against.

Key references:
- PURSE — conformal keypoints + geometric propagation, <https://arxiv.org/abs/2303.12246>
- CLOSURE — fast pose uncertainty sets, <https://www.roboticsproceedings.org/rss20/p072.pdf>
- SLUE — minimum-volume bounding ellipsoid for pose, <https://arxiv.org/abs/2511.21666>
- SPNv3 — flight-ready spacecraft pose, <https://arxiv.org/abs/2409.11661>
- SPEED+ — the lightbox/sunlamp illumination domains, <https://arxiv.org/abs/2110.03101>

### Run the moment-fit baseline too

Marginal cost is a covariance computation on errors already collected. Payoff is
the paper's central figure: the 2-sigma moment ellipsoid contains the truth well
below its nominal rate under non-Gaussian error, while the conformal set hits
its target by construction.

### Open decision: Mondrian vs global conformal

**Must be settled before rendering**, because it changes how many calibration
samples each cell of the sweep needs.

Standard conformal gives *marginal* coverage — correct on average over the whole
state space. Slice by range afterwards and the far-range bins are under-covered,
exactly where the sets matter most. Mondrian (group-conditional) conformal
calibrates separately within each (range, aspect, sun-angle) bin and avoids this.

### Known gap to acknowledge in writing

The theory assumes a **hard** bound (`eps` in a compact set). Conformal gives a
**high-probability** bound (coverage >= 1 - alpha). Either set alpha small and
state that guarantees hold with that probability, or restate the assumption as
"compact with probability 1 - alpha" and carry alpha through. A reviewer who
knows conformal prediction will notice, so address it explicitly.

## Renderer

**UE5 for both modes**, despite lower throughput than the alternatives.

The tempting split is BlenderProc or Isaac Replicator for the offline sweep and
UE for the demo. Rejected: two renderers means two domain gaps, so the oracle
fitted in one would not describe the errors the other exhibits. One renderer,
one gap, consistent statistics.

Throughput is survivable — UE5 path tracer at 512x512 is roughly 0.5-3 s/frame
on a decent RTX card, so 1e4 frames is overnight and 1e5 is a weekend.

| | Mode A: oracle fitting | Mode B: closed loop |
|---|---|---|
| Purpose | measure error vs state | the video |
| Volume | 1e4-1e6 frames | 1e2-1e3 |
| Rate | throughput, headless, offline | ~1-10 Hz |
| Tooling | editor Python + Movie Render Queue | ZeroMQ + C++ TCP actor |
| Paper role | the contribution | optional |

### UE gotchas

- **Do not use Nanite.** Cluster LOD silently moves keypoint vertices between
  renders, so 3D landmark coordinates drift and the labels quietly go wrong.
  Path-tracer support is also fragile. Use a static non-Nanite mesh.
- **UE's built-in Python is editor-only** — it does not exist in a packaged
  build. Mode B either runs in-editor under PIE, or needs a small C++ TCP actor.
- Use the **path tracer, not Lumen**, for the dataset. Lumen's specular handling
  on MLI is not trustworthy, and MLI is the whole point.
- Polycount 300k-1.5M triangles for the target.

## Target asset

**No Tango/PRISMA mesh is publicly obtainable.** SPEED/SPEED+ ship rendered
images and pose labels only; the CAD is proprietary to the PRISMA industrial
team. The PoliMi Tango dataset (Zenodo 6499008, CC BY-NC 4.0) is images + JSON
labels, no mesh. "Same target as the benchmark" is not achievable — drop it as a
criterion. Benchmarking against published SPEED+ *numbers* is still fine.

### Plan: validate on SwissCube, publish on a NASA observatory

**SwissCube** (<https://huggingface.co/datasets/EPFL-CVLAB-SPACECRAFT/SwissCube>)
is CAD-accurate to the screws and ships a matched 50k-image published benchmark,
so the whole pipeline — renderer, keypoint net, conformal calibration, SLUE —
can be verified against someone else's published numbers before trusting a
self-fitted oracle. Its lack of MLI area does not matter for a validation target.
Licence: HuggingFace repo tag says MIT; **verify the actual LICENSE file**.

**NASA 3D Resources** (<https://github.com/nasa/NASA-3D-Resources>) — LRO or SDO
— for the actual study. Pull from the GitHub mirror for source formats, not the
`.glb`-only web portal. Licence is the cleanest available: "free and without
copyright", subject to NASA media guidelines (attribute, no implied endorsement,
insignia not public domain).

Rejected: SPE3R (CC BY-NC-SA, ShareAlike is viral, and meshes are "watertighted"
which thickens or destroys thin booms — bad for keypoints); ESA SciFleet
(licence unverifiable, site inaccessible); CGTrader/TurboSquid (renders in papers
are fine but the mesh cannot be redistributed with the paper, and listings are
mostly stylized).

**Nothing ships mesh + pipeline together.** SPE3R is closest but does not
distribute its UE project; URSO promised a simulator in 2019 and never delivered.
Budget for building the UE5 scene; you are only sourcing geometry.

### MLI is a shader problem, not a purchase problem

No asset at any price ships convincing multi-layer insulation, so "must author
MLI" costs nothing in relative terms between candidates. Authoring recipe:

- metallic 1.0; gold-Kapton base colour ~ (1.0, 0.77, 0.34), or near-white for
  beta cloth
- **spatially varying** roughness 0.12-0.40 driven by a crinkle mask
- high-frequency tiling crinkle normal blended with a low-frequency sag normal
- anisotropy, plus a subtle thin-film/iridescence tint — the grazing-angle colour
  shift is a real, learnable cue
- 4K textures on MLI panels, 2K elsewhere
- **16-bit normal maps on MLI.** 8-bit banding produces stair-stepped specular
  glints that a network will overfit as fake keypoints.
- path tracer at 512-2048 spp; MLI's near-mirror lobe is the slowest-converging
  part of the scene and undersampled fireflies would poison calibration data

## Phasing

| | Deliverable | Status |
|---|---|---|
| P0 | `EmpiricalOracle` behind the existing `SetOracle` interface | interface already frozen |
| P1 | UE scene: target, sun, Earth albedo, camera | asset chosen, not started |
| P2 | Sweep harness: `UnrealEditor-Cmd -ExecutePythonScript`, MRQ, BOP-format output | not started |
| P3 | Keypoint net trained on the sweep | stack chosen |
| P4 | Per-bin conformal + SLUE, and the moment-fit baseline | design settled except Mondrian question |
| P5 | Re-run planner with `EmpiricalOracle`, compare | the paper result |
| P6 | ZeroMQ bridge + closed loop | optional |

## Interface contract

`EmpiricalOracle` must satisfy the existing `SetOracle` interface:

```python
radii(p)                 -> (sigma, V)                 # numpy
radii_cvx(p_expr, p_ref) -> [convex cvxpy expressions] # must be DCP-convex
```

The DCP-convexity requirement means the fit **cannot be an arbitrary regressor** —
it must be a convex envelope of the measured radii. Options, increasing effort:

1. Quadratic-plus-nonnegative least squares:
   `sigma(p) = a||z-p||^2 + b||z-p|| + c||p-p*||^2 + d`, all coefficients >= 0.
   Convex by construction, ~5 parameters. Start here.
2. Max-of-affine: `sigma(p) = max_k(a_k' phi(p) + b_k)`, still DCP via
   `cvx.maximum`.
3. Input-convex neural network — most expressive, but the DCP expression must be
   hand-written.

This is precisely the enveloping function
`Lambda_hat(x) = inf{ g(x) : g convex, g >= Lambda(x) }` of Eq. (13) in the
published JGCD paper, which explicitly left its construction "to the
practitioner". Building it for a learned estimator is a real contribution, not
plumbing.

## Physics that actually changes the oracle

Not decoration — these are why the error is state-dependent:

- Sun as a directional light, correct irradiance, very hard shadows, no
  atmospheric scatter
- Earth albedo as a large area light / HDRI — significant secondary illumination
  in LEO, and what fills the terminator shadows
- Black background, star field, solar panels, antennas
- HDR capture with realistic camera response (exposure, saturation, bloom),
  sensor noise, motion blur
- Eclipse / terminator passes as the extreme case
