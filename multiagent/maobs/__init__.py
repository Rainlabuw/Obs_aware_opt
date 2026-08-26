"""Estimation-aware multi-agent planning with set-valued measurement uncertainty.

Reference implementation for the multi-agent extension of

    A. Deole and M. Mesbahi, "Estimation-Aware Trajectory Optimization with
    Set-Valued Measurement Uncertainties", JGCD.

The package is deliberately free of any plotting or visualisation: every entry
point returns arrays.  Figure generation lives in ``experiments/figures.py``.
"""

from .dynamics import LinearAgent, clohessy_wiltshire, planar_double_integrator
from .estimator import EstimationStats, monte_carlo
from .observability import (
    degree_of_observability,
    degree_of_network_observability,
    fuse,
    informativity_ratio,
    is_informative,
    network_shapes,
    total_directional_radius,
    volume_from_shape,
)
from .oracle import MonocularPoseOracle, RangeBearingOracle, SetOracle
from .scvx import PlannerConfig, PlannerResult, plan

__all__ = [
    "LinearAgent",
    "planar_double_integrator",
    "clohessy_wiltshire",
    "SetOracle",
    "RangeBearingOracle",
    "MonocularPoseOracle",
    "degree_of_observability",
    "degree_of_network_observability",
    "total_directional_radius",
    "volume_from_shape",
    "fuse",
    "is_informative",
    "informativity_ratio",
    "network_shapes",
    "PlannerConfig",
    "PlannerResult",
    "plan",
    "monte_carlo",
    "EstimationStats",
]
