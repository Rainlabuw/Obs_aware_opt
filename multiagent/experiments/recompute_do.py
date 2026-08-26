"""Recompute the degree of observability at a common probe size.

``D_O`` depends on the probe size ``epsilon``, and it is only the clean affine
image of the total directional radius while ``epsilon`` exceeds every fused set
radius encountered.  A single-agent monocular fleet has a much larger fused set
than a three-agent one, so a probe size chosen for ``m = 3`` gets clamped at
``m = 1`` and the ablation column stops being comparable.

Rather than re-planning, this script reloads the stored agent positions,
rebuilds the fused shape matrices through the oracle, picks one ``epsilon``
that is safe for *every* case, and rewrites the ``D_O`` entries in the JSON
summary.  Planning is deterministic, so this is exact.
"""

from __future__ import annotations

import json

import numpy as np

import scenario_satellite as S
from common import RESULTS

from maobs import degree_of_observability, total_directional_radius  # noqa: E402
from maobs.observability import network_shapes  # noqa: E402


def fused_for(positions: np.ndarray):
    m = positions.shape[0]
    _, oracles, _, _ = S.make(m)
    fused, _ = network_shapes(oracles, positions)
    return fused


def main() -> None:
    npz = np.load(RESULTS / "satellite.npz")
    with open(RESULTS / "satellite.json") as fh:
        meta = json.load(fh)

    entries = []  # (row, fused)
    for row in meta[0]["main"]:
        entries.append((row, fused_for(npz[f"{row['case']}_positions"])))
    for row in meta[0]["ablation"]:
        entries.append((row, fused_for(npz[f"ablation_m{row['m']}_positions"])))

    max_radius = max(
        float(np.sqrt(np.max(np.diag(Q)))) for _, fused in entries for Q in fused
    )
    epsilon = float(np.ceil(2.0 * max_radius / 5.0) * 5.0)
    print(f"largest fused set radius over all cases : {max_radius:.3f} m")
    print(f"common probe size epsilon               : {epsilon:.1f} m "
          f"(>= 2x, so no case is clamped)\n")

    for row, fused in entries:
        tdr = total_directional_radius(fused)
        do = degree_of_observability(fused, epsilon)
        affine = 2.0 * (3 * len(fused) * epsilon - tdr)
        assert np.isclose(do, affine), "clamping still active -- raise epsilon"
        row["D_O"] = do
        row["total_directional_radius"] = tdr
        row["epsilon"] = epsilon
        row["max_fused_radius"] = float(
            max(np.sqrt(np.max(np.diag(Q))) for Q in fused)
        )
        print(f"  {row['case']:<12s} m={row['m']}  "
              f"max_radius={row['max_fused_radius']:6.3f}  "
              f"TDR={tdr:8.2f}  D_O={do:9.2f}")

    meta[0]["epsilon_note"] = (
        f"D_O evaluated at a common probe size epsilon = {epsilon:.1f} m, "
        "which exceeds every fused set radius encountered, so D_O is exactly "
        "affine and strictly decreasing in the total directional radius."
    )
    with open(RESULTS / "satellite.json", "w") as fh:
        json.dump(meta, fh, indent=2)
    print(f"\nrewrote {RESULTS / 'satellite.json'}")


if __name__ == "__main__":
    main()
