"""
Target Velocity Profiler based on track curvature.
Calculates maximum safe cornering speeds subject to lateral acceleration limits.
"""

import math  # noqa: F401


class VelocityProfiler:
    """Generates target speed profiles based on track curvature or precomputed data."""

    def __init__(self, default_speed=4.0, max_speed=8.0, max_lat_accel=5.0):
        self.default_speed = default_speed
        self.max_speed = max_speed
        self.max_lat_accel = max_lat_accel

    def compute_target_speed(self, kappa, fallback_speed=None):
        """Calculates curvature-limited velocity: v_max = sqrt(a_lat_max / |kappa|)."""
        # ======================================================================
        # TODO: Milestone 5.1 — Curvature-Limited Velocity Profiler
        #
        # 1. Curvature & Lateral Grip Limit:
        #    Recall equation of centripetal acceleration.
        #    Compute the physical velocity ceiling:
        #      - On straight sections (|kappa| -> 0), allow self.max_speed.
        #      - In curved corners (|kappa| > 0), limit velocity by tire grip:
        #        use the curvature-limited speed formula.
        #
        # 2. Clamping & Bounds:
        #    - Ensure target speed does not exceed self.max_speed.
        #    - If a fallback_speed is provided, respect it as an upper limit.
        #    - Maintain a minimum crawl speed (e.g. 1.5 m/s) to prevent the vehicle
        #      from stalling or coming to a complete stop inside tight turns.
        #    - Return the resulting target velocity as a float.
        # ======================================================================
        pass
