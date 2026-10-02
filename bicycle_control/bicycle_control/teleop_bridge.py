"""
Teleoperation bridge node:
Subscribes to standard geometry_msgs/Twist on /cmd_vel (from teleop_twist_keyboard or joy)
and translates it to /throttle (Float32 in [-1.0, 1.0]) and /steer (Float32 in radians).

Supports two progression phases:
- Phase 1 (Milestone 3): Open-loop feedforward mapping with a safety watchdog timer.
- Phase 2 (Milestone 4): Closed-loop speed regulation using PIDLongitudinalController.
"""

import numpy as np  # noqa: F401
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# ==============================================================================
# Phase 2 (Milestone 4): Uncomment these imports when upgrading to cruise control
# ==============================================================================
# from nav_msgs.msg import Odometry
# from bicycle_control.longitudinal_pid import PIDLongitudinalController


class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge')
        self.get_logger().info('Teleoperation Bridge Node Initialized')

        # Parameters
        self.declare_parameter('max_linear_vel', 5.0)     # m/s corresponding to full 1.0 throttle
        self.declare_parameter('max_angular_vel', 1.0)    # rad/s corresponding to full steering
        self.declare_parameter('max_steer_rad', 0.610865)  # radians (~35 degrees)
        self.declare_parameter('auto_zero_timeout', 0.5)  # seconds before zeroing commands
        self.declare_parameter('use_cruise_control', False)  # Enable in Milestone 4.2

        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.max_steer_rad = float(self.get_parameter('max_steer_rad').value)
        self.auto_zero_timeout = float(self.get_parameter('auto_zero_timeout').value)
        self.use_cruise_control = bool(self.get_parameter('use_cruise_control').value)

        # Publishers (10 Hz rate per assignment specification)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 10)
        self.steer_pub = self.create_publisher(Float32, '/steer', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.current_throttle = 0.0
        self.current_steer = 0.0
        self.target_vel = 0.0
        self.last_cmd_time = self.get_clock().now()

        # ======================================================================
        # Phase 2 (Milestone 4.2): Closed-Loop Cruise Control Setup
        #
        # When self.use_cruise_control is True:
        # 1. Instantiate PIDLongitudinalController(kp=1.0, ki=0.2, kd=0.05, dt=0.1)
        # 2. Subscribe to Odometry on '/state' with callback self.odom_callback
        # 3. Maintain self.current_vel (initialized to 0.0)
        # ======================================================================

        # Publish loop at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_commands)

    # def odom_callback(self, msg: Odometry):
    #     """Milestone 4.2: Extracts vehicle forward speed from /state odometry."""
    #     pass

    def cmd_callback(self, msg: Twist):
        """Translates Twist linear.x to throttle [-1, 1] and angular.z into steering (rad)."""
        # ======================================================================
        # TODO: Milestone 3.1 — Teleoperation Command Mapping
        #
        # 1. Command Timestamp:
        #    Record the receipt time of this message in self.last_cmd_time.
        #
        # 2. Steering Mapping (Angular Velocity -> Steering Angle):
        #    - Normalize msg.angular.z relative to self.max_angular_vel.
        #    - Scale to steering radians using self.max_steer_rad.
        #    - Constrain steering to [-self.max_steer_rad, self.max_steer_rad]
        #      and store in self.current_steer.
        #
        # 3. Throttle / Velocity Mapping:
        #    - In Open-Loop Mode (Milestone 3):
        #        Normalize msg.linear.x relative to self.max_linear_vel.
        #        Constrain effort to [-1.0, 1.0] and store in self.current_throttle.
        #
        #    - In Closed-Loop Cruise Control Mode (Milestone 4.2 Upgrade):
        #        Instead of raw throttle, treat msg.linear.x as the target velocity
        #        setpoint v_target (store in self.target_vel).
        # ======================================================================
        pass

    def publish_commands(self):
        """Periodically publishes throttle and steering commands at 10 Hz."""
        # ======================================================================
        # TODO: Milestone 3.2 — Safety Watchdog & Command Publishing
        #
        # 1. Watchdog Timer:
        #    - Compute the elapsed time (in seconds) since self.last_cmd_time.
        #    - If elapsed time exceeds self.auto_zero_timeout:
        #        Zero out active commands (throttle, steering, target velocity)
        #        to prevent a runaway vehicle if communication is lost.
        #        (If using cruise control, also reset the PID integrator).
        #
        # 2. Throttle Generation (Milestone 4.2 Closed-Loop only):
        #    - If cruise control is active:
        #        Compute self.current_throttle using your PIDLongitudinalController
        #        given self.target_vel and self.current_vel.
        #
        # 3. ROS Telemetry Publishing:
        #    - Construct Float32 messages for current throttle and steering.
        #    - Publish to self.throttle_pub and self.steer_pub.
        # ======================================================================
        pass


def main(args=None):
    rclpy.init(args=args)
    bridge = TeleopBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
