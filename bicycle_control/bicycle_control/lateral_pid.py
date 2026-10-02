"""
High-Level Lateral Steering Controller: Reactive Lateral PID.
Steers based on instantaneous Cross-Track Error (CTE) and Heading Error.
"""

import math
import numpy as np  # noqa: F401


class LateralPIDController:
    """Lateral PID steering controller based on Cross-Track Error (CTE) and Heading Error.

    Commands front wheel steering based on instantaneous lateral offset (cross-track error)
    and orientation error relative to the nearest path waypoint.
    """

    def __init__(self, kp=0.8, ki=0.02, kd=0.15, k_yaw=0.5, dt=0.1,
                 max_steer_rad=math.radians(35.0), integral_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.k_yaw = k_yaw
        self.dt = dt
        self.max_steer_rad = max_steer_rad
        self.integral_limit = integral_limit

        self.integral_cte = 0.0
        self.prev_cte = 0.0

    def compute_steering(self, cte, heading_err):
        """Computes front wheel steering angle delta in radians.

        Args:
            cte: Signed cross-track error in meters (positive = vehicle is left of path).
            heading_err: Heading error in radians (psi_vehicle - psi_path).

        Returns:
            delta_rad: Commanded front steering angle in radians [-max_steer_rad, max_steer_rad].
        """
        # ======================================================================
        # TODO: Milestone 5.2 — Reactive Lateral PID Controller
        #
        # 1. Error Sign Conventions:
        #    - Positive CTE means the vehicle is to the LEFT of the reference path.
        #      To drive back toward the centerline, steer RIGHT (negative delta).
        #    - Positive heading error means the vehicle is yawed LEFT of the path tangent.
        #      To align parallel with the path, steer RIGHT (negative delta).
        #
        # 2. Integrator & Anti-Windup:
        #    - Integrate lateral tracking error over time step self.dt.
        #    - Apply anti-windup clamping to [-self.integral_limit, self.integral_limit].
        #
        # 3. Finite Difference Derivative:
        #    - Calculate the rate of change of lateral error with respect to self.dt.
        #    - Store the current error in self.prev_cte for the next cycle.
        #
        # 4. Steering Command & Actuator Saturation:
        #    - Combine proportional, integral, and derivative terms for lateral tracking,
        #      along with proportional damping on heading error (k_yaw * heading_err).
        #    - Clamp the resulting steering angle to [-self.max_steer_rad, self.max_steer_rad].
        #    - Return the commanded steering angle in radians.
        # ======================================================================
        pass

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral_cte = 0.0
        self.prev_cte = 0.0
