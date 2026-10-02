"""
Low-Level Powertrain Cruise Controller (Longitudinal PID).
Regulates vehicle speed via normalized throttle/braking effort.
"""

import numpy as np  # noqa: F401


class PIDLongitudinalController:
    """Low-Level Powertrain Cruise Controller / Electronic Speed Control (ESC).

    Translates high-level velocity requests into normalized throttle/brake effort.
    Because physical vehicles experience friction and speed-squared aerodynamic drag,
    a closed-loop speed regulator is required to maintain target velocity.
    """

    def __init__(self, kp=1.0, ki=0.2, kd=0.05, dt=0.1,
                 max_throttle=1.0, max_brake=1.0, integral_limit=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target_vel, current_vel):
        """Computes normalized throttle/braking effort in [-1.0, 1.0]."""
        # ======================================================================
        # TODO: Milestone 4.1 — Longitudinal PID Speed Control & Anti-Windup
        #
        # 1. Velocity Error:
        #    Compute tracking error between target velocity and current velocity.
        #
        # 2. Integrator & Anti-Windup:
        #    Accumulate error into self.integral over time step self.dt.
        #    Clamp the accumulated integral to [-self.integral_limit, self.integral_limit]
        #    to prevent actuator saturation and overshoot.
        #
        # 3. Finite Difference Derivative:
        #    Compute the rate of change of error with respect to self.dt.
        #    Update self.prev_error for the next control cycle.
        #
        # 4. Control Output & Actuator Saturation:
        #    Compute the PID control signal: u = P + I + D.
        #    Constrain output effort between [-self.max_brake, self.max_throttle].
        #    Return the resulting float control effort.
        # ======================================================================
        pass

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral = 0.0
        self.prev_error = 0.0
