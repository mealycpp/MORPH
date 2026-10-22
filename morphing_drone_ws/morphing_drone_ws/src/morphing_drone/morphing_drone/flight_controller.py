#!/usr/bin/env python3
"""
Flight Controller for Morphing Drone
-------------------------------------
Implements cascaded PID control:
  Outer loop: position → desired attitude
  Inner loop: attitude → motor thrust commands

Publishes wrench commands to each EDF via Gazebo force plugin.
Subscribes to ground truth odometry + IMU for state feedback.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String, Bool


class PIDController:
    """Simple PID with anti-windup."""

    def __init__(self, kp, ki, kd, max_integral=10.0,
                 output_min=-float('inf'), output_max=float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error, current_time):
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return 0.0

        dt = current_time - self.prev_time
        if dt <= 0:
            return 0.0

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.max_integral, self.max_integral)
        i_term = self.ki * self.integral

        # Derivative
        d_term = self.kd * (error - self.prev_error) / dt

        self.prev_error = error
        self.prev_time = current_time

        output = p_term + i_term + d_term
        return np.clip(output, self.output_min, self.output_max)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None


def quaternion_to_euler(q):
    """Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)."""
    x, y, z, w = q
    # Roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(np.clip(sinp, -1.0, 1.0))
    # Yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class FlightController(Node):
    """
    Main flight controller node.

    Computes per-EDF thrust commands to maintain hover while accounting
    for the current morphing configuration (arm geometry changes the
    effective moment arms).
    """

    def __init__(self):
        super().__init__('flight_controller')

        # ---------- Parameters ----------
        self.declare_parameter('target_altitude', 5.0)
        self.declare_parameter('drone_mass', 2.5)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('max_thrust_per_edf', 17.86)
        self.declare_parameter('arm_inner_len', 0.159)
        self.declare_parameter('arm_outer_len', 0.159)
        self.declare_parameter('control_rate', 100.0)

        self.target_alt = self.get_parameter('target_altitude').value
        self.mass = self.get_parameter('drone_mass').value
        self.g = self.get_parameter('gravity').value
        self.max_thrust = self.get_parameter('max_thrust_per_edf').value
        self.arm_inner = self.get_parameter('arm_inner_len').value
        self.arm_outer = self.get_parameter('arm_outer_len').value
        rate = self.get_parameter('control_rate').value

        # Weight that needs to be overcome for hover
        self.hover_thrust_total = self.mass * self.g
        self.hover_thrust_per_edf = self.hover_thrust_total / 4.0

        # ---------- PID Controllers ----------
        # Altitude PID outputs a CORRECTION around hover thrust
        # Very gentle gains to prevent overshoot
        self.alt_pid = PIDController(1.0, 0.1, 2.0, max_integral=3.0,
                                     output_min=-self.hover_thrust_total * 0.4,
                                     output_max=self.hover_thrust_total * 0.4)
        self.roll_pid = PIDController(3.0, 0.2, 1.5, max_integral=1.0,
                                      output_min=-5.0, output_max=5.0)
        self.pitch_pid = PIDController(3.0, 0.2, 1.5, max_integral=1.0,
                                       output_min=-5.0, output_max=5.0)
        self.yaw_pid = PIDController(2.0, 0.1, 0.8, max_integral=1.0,
                                     output_min=-3.0, output_max=3.0)

        # ---------- State ----------
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation_euler = np.zeros(3)  # roll, pitch, yaw
        self.angular_velocity = np.zeros(3)
        self.state_received = False
        self.flight_enabled = False  # Controlled by morph sequence

        # Current servo angles (updated by morph_controller)
        # These affect the moment arms for mixing
        self.servo_angles_rad = np.zeros(4)

        # Nominal arm headings in X-config (radians)
        self.base_headings = np.array([
            math.pi / 4,       # Arm 1: front-right
            3 * math.pi / 4,   # Arm 2: front-left
            5 * math.pi / 4,   # Arm 3: rear-left
            7 * math.pi / 4,   # Arm 4: rear-right
        ])

        # EDF spin directions for yaw control (1 = CW, -1 = CCW)
        self.spin_dirs = np.array([1, -1, 1, -1])

        # ---------- Publishers (wrench to each EDF) ----------
        self.thrust_pubs = []
        for i in range(1, 5):
            pub = self.create_publisher(
                Wrench,
                f'/morphing_drone/edf_{i}/thrust_cmd',
                10
            )
            self.thrust_pubs.append(pub)

        # Publish motor commands for data logging
        self.motor_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/morphing_drone/motor_commands',
            10
        )

        # ---------- Subscribers ----------
        self.create_subscription(
            Odometry,
            '/morphing_drone/ground_truth/odom',
            self._odom_callback,
            10
        )
        self.create_subscription(
            Imu,
            '/morphing_drone/imu/data',
            self._imu_callback,
            10
        )
        self.create_subscription(
            Float64MultiArray,
            '/morphing_drone/servo_angles',
            self._servo_callback,
            10
        )
        self.create_subscription(
            Bool,
            '/morphing_drone/flight_enabled',
            self._flight_enabled_callback,
            10
        )
        # Track config changes for post-morph logging
        self.create_subscription(
            String,
            '/morphing_drone/current_config',
            self._config_callback,
            10
        )
        self._last_config = 'X'
        self._morph_log_pending = False
        self._morph_log_time = 0.0

        # Waypoint target (optional — only used in obstacle course mode)
        # Published as Float64MultiArray with [x, y, z] world-frame target
        from std_msgs.msg import Float64MultiArray as _FMA
        self.target_x = 0.0
        self.target_y = 0.0
        self.has_waypoint = False
        self.create_subscription(
            _FMA,
            '/morphing_drone/target_position',
            self._target_pos_callback,
            10
        )

        # ---------- Control loop timer ----------
        period = 1.0 / rate
        self.timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'Flight controller started — hover thrust/EDF: '
            f'{self.hover_thrust_per_edf:.2f} N, target alt: {self.target_alt} m'
        )

    def _odom_callback(self, msg):
        p = msg.pose.pose.position
        self.position = np.array([p.x, p.y, p.z])

        v = msg.twist.twist.linear
        self.velocity = np.array([v.x, v.y, v.z])

        q = msg.pose.pose.orientation
        self.orientation_euler = np.array(
            quaternion_to_euler((q.x, q.y, q.z, q.w))
        )
        self.state_received = True

    def _imu_callback(self, msg):
        av = msg.angular_velocity
        self.angular_velocity = np.array([av.x, av.y, av.z])

    def _servo_callback(self, msg):
        if len(msg.data) == 4:
            self.servo_angles_rad = np.array(msg.data)

    def _flight_enabled_callback(self, msg):
        if msg.data and not self.flight_enabled:
            self.get_logger().info('Flight ENABLED — taking off')
            self.alt_pid.reset()  # Reset PID to avoid integral windup
        elif not msg.data and self.flight_enabled:
            self.get_logger().info('Flight DISABLED — motors off')
        self.flight_enabled = msg.data

    def _config_callback(self, msg):
        # When the morph controller switches config, schedule an immediate
        # diagnostic log a bit after the morph completes (~3.5s)
        if msg.data != self._last_config:
            self._last_config = msg.data
            self._morph_log_pending = True
            self._morph_log_time = self.get_clock().now().nanoseconds * 1e-9 + 3.5

    def _target_pos_callback(self, msg):
        # Expects Float64MultiArray with [x, y, z]
        if len(msg.data) >= 3:
            self.target_x = msg.data[0]
            self.target_y = msg.data[1]
            self.target_alt = msg.data[2]
            self.has_waypoint = True

    def _compute_edf_positions(self):
        """
        Compute each EDF's (x, y) position relative to base_link center,
        accounting for current servo angles.
        """
        positions = []
        for i in range(4):
            heading = self.base_headings[i]
            servo_angle = self.servo_angles_rad[i]

            # Inner segment endpoint (servo location)
            sx = self.arm_inner * math.cos(heading)
            sy = self.arm_inner * math.sin(heading)

            # Outer segment endpoint (EDF location)
            # The outer arm heading = base heading + servo deflection
            outer_heading = heading + servo_angle
            ex = sx + self.arm_outer * math.cos(outer_heading)
            ey = sy + self.arm_outer * math.sin(outer_heading)

            positions.append(np.array([ex, ey]))

        return positions

    def _mixer(self, total_thrust, roll_cmd, pitch_cmd, yaw_cmd):
        """
        Mix altitude, roll, pitch, yaw commands into per-EDF thrust.

        Uses current EDF positions (which change with morphing) to compute
        the correct mixing matrix dynamically.
        """
        edf_pos = self._compute_edf_positions()

        # Build mixing matrix:
        # For each EDF i, thrust T_i contributes:
        #   - Total thrust: +T_i (Z direction)
        #   - Roll torque:  T_i * y_i (moment about X axis)
        #   - Pitch torque: -T_i * x_i (moment about Y axis)
        #   - Yaw torque:   spin_dir_i * torque_coeff * T_i
        torque_coeff = 0.01  # Ratio of reaction torque to thrust

        # We solve: [total_thrust, roll, pitch, yaw] = M * [T1, T2, T3, T4]
        M = np.zeros((4, 4))
        for i in range(4):
            M[0, i] = 1.0                          # Thrust
            M[1, i] = edf_pos[i][1]                 # Roll (y position)
            M[2, i] = -edf_pos[i][0]                # Pitch (-x position)
            M[3, i] = self.spin_dirs[i] * torque_coeff  # Yaw

        desired = np.array([total_thrust, roll_cmd, pitch_cmd, yaw_cmd])

        # Solve using pseudoinverse (handles non-square or singular cases)
        try:
            thrusts = np.linalg.lstsq(M, desired, rcond=None)[0]
        except np.linalg.LinAlgError:
            thrusts = np.full(4, total_thrust / 4.0)

        # Clamp to valid thrust range
        thrusts = np.clip(thrusts, 0.0, self.max_thrust)

        return thrusts

    def _control_loop(self):
        if not self.state_received:
            if not hasattr(self, '_wait_count'):
                self._wait_count = 0
            self._wait_count += 1
            if self._wait_count % 200 == 1:
                self.get_logger().warn(
                    'Waiting for odometry on /morphing_drone/ground_truth/odom...'
                )
            return

        # If flight is not enabled, publish zero thrust (drone stays grounded)
        if not self.flight_enabled:
            zero_thrusts = np.zeros(4)
            for i in range(4):
                wrench = Wrench()
                wrench.force.z = 0.0
                self.thrust_pubs[i].publish(wrench)
            cmd_msg = Float64MultiArray()
            cmd_msg.data = zero_thrusts.tolist()
            self.motor_cmd_pub.publish(cmd_msg)
            return

        import time as _time
        t = _time.monotonic()

        alt_error = self.target_alt - self.position[2]
        vel_z = self.velocity[2]

        # PD controller with explicit velocity damping (no integral)
        # P: proportional to altitude error (clamped)
        # D: damps vertical velocity to prevent overshoot
        alt_error_clamped = np.clip(alt_error, -2.0, 2.0)

        kp = 1.5   # N per meter of error
        kd = 4.0   # N per m/s of vertical velocity (heavy damping)

        alt_correction = kp * alt_error_clamped - kd * vel_z

        # Clamp correction to ±30% of hover thrust
        max_correction = self.hover_thrust_total * 0.3
        alt_correction = np.clip(alt_correction, -max_correction, max_correction)

        # Total = gravity feedforward + PD correction
        thrust_total = self.hover_thrust_total + alt_correction

        # Hard clamp
        thrust_total = np.clip(thrust_total, 0.0, self.hover_thrust_total * 1.4)

        # Equal thrust to all 4 motors
        thrust_per_motor = thrust_total / 4.0
        motor_thrusts = np.full(4, thrust_per_motor)

        # Publish to individual EDF topics
        for i in range(4):
            wrench = Wrench()
            wrench.force.z = float(thrust_per_motor)
            self.thrust_pubs[i].publish(wrench)

        # Publish for bridge and logger
        cmd_msg = Float64MultiArray()
        cmd_msg.data = motor_thrusts.tolist()
        self.motor_cmd_pub.publish(cmd_msg)

        # Log every 5 seconds, OR immediately after a morph completes
        if not hasattr(self, '_log_count'):
            self._log_count = 0
        self._log_count += 1
        now_t = self.get_clock().now().nanoseconds * 1e-9
        morph_done = self._morph_log_pending and now_t >= self._morph_log_time
        if self._log_count % 500 == 0 or morph_done:
            tag = f'[POST-MORPH → {self._last_config}] ' if morph_done else ''
            servo_deg = [math.degrees(a) for a in self.servo_angles_rad]
            self.get_logger().info(
                f'{tag}Alt: {self.position[2]:.2f}m (err: {alt_error:+.2f}) | '
                f'Vz: {vel_z:+.2f} | '
                f'Servos: [{servo_deg[0]:+.0f},{servo_deg[1]:+.0f},'
                f'{servo_deg[2]:+.0f},{servo_deg[3]:+.0f}]° | '
                f'T: [FR={motor_thrusts[0]:.2f}, FL={motor_thrusts[1]:.2f}, '
                f'RL={motor_thrusts[2]:.2f}, RR={motor_thrusts[3]:.2f}]N'
            )
            if morph_done:
                self._morph_log_pending = False


def main(args=None):
    rclpy.init(args=args)
    node = FlightController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
