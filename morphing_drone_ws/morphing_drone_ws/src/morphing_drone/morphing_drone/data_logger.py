#!/usr/bin/env python3
"""
Data Logger — Records flight data at 50 Hz for each morphing configuration.

Subscribes to odometry, IMU, motor commands, and current config.
Writes timestamped CSV with all flight metrics.
"""

import os
import csv
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String


def quaternion_to_euler(q):
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(np.clip(sinp, -1.0, 1.0))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class DataLogger(Node):

    def __init__(self):
        super().__init__('data_logger')

        self.declare_parameter('output_dir', os.path.expanduser('~/morphing_drone_data'))
        self.declare_parameter('log_rate_hz', 50.0)

        self.output_dir = self.get_parameter('output_dir').value
        log_rate = self.get_parameter('log_rate_hz').value

        os.makedirs(self.output_dir, exist_ok=True)

        # State variables
        self.position = np.zeros(3)
        self.velocity_linear = np.zeros(3)
        self.velocity_angular = np.zeros(3)
        self.orientation_euler = np.zeros(3)
        self.accel_linear = np.zeros(3)
        self.motor_commands = np.zeros(4)
        self.current_config = 'X'
        self.start_time = None
        self.data_ready = False

        # CSV file
        self.csv_path = os.path.join(self.output_dir, 'flight_data.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp_s', 'config',
            'pos_x', 'pos_y', 'pos_z',
            'vel_x', 'vel_y', 'vel_z',
            'roll_rad', 'pitch_rad', 'yaw_rad',
            'ang_vel_x', 'ang_vel_y', 'ang_vel_z',
            'accel_x', 'accel_y', 'accel_z',
            'motor_1_N', 'motor_2_N', 'motor_3_N', 'motor_4_N',
            'total_thrust_N', 'estimated_power_W',
            'stability_index',
        ])

        # Subscribers
        self.create_subscription(
            Odometry,
            '/morphing_drone/ground_truth/odom',
            self._odom_cb, 10
        )
        self.create_subscription(
            Imu,
            '/morphing_drone/imu/data',
            self._imu_cb, 10
        )
        self.create_subscription(
            Float64MultiArray,
            '/morphing_drone/motor_commands',
            self._motor_cb, 10
        )
        self.create_subscription(
            String,
            '/morphing_drone/current_config',
            self._config_cb, 10
        )

        # Log timer
        period = 1.0 / log_rate
        self.timer = self.create_timer(period, self._log_data)

        self.get_logger().info(f'Data logger started → {self.csv_path}')

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.position = np.array([p.x, p.y, p.z])

        v = msg.twist.twist.linear
        self.velocity_linear = np.array([v.x, v.y, v.z])

        q = msg.pose.pose.orientation
        self.orientation_euler = np.array(
            quaternion_to_euler((q.x, q.y, q.z, q.w))
        )

        av = msg.twist.twist.angular
        self.velocity_angular = np.array([av.x, av.y, av.z])
        self.data_ready = True

    def _imu_cb(self, msg):
        a = msg.linear_acceleration
        self.accel_linear = np.array([a.x, a.y, a.z])

    def _motor_cb(self, msg):
        if len(msg.data) == 4:
            self.motor_commands = np.array(msg.data)

    def _config_cb(self, msg):
        self.current_config = msg.data

    def _log_data(self):
        if not self.data_ready:
            return

        t = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = t

        elapsed = t - self.start_time
        total_thrust = np.sum(self.motor_commands)

        # Estimate power: P ≈ T^1.5 / (efficiency * sqrt(disk_area))
        # Simplified: ~54 W per N for this size EDF
        estimated_power = np.sum(
            np.abs(self.motor_commands) * 54.0  # W/N estimate
        )

        # Stability index: RMS of roll + pitch deviation
        stability = math.sqrt(
            self.orientation_euler[0] ** 2 + self.orientation_euler[1] ** 2
        )

        self.csv_writer.writerow([
            f'{elapsed:.4f}',
            self.current_config,
            f'{self.position[0]:.4f}', f'{self.position[1]:.4f}',
            f'{self.position[2]:.4f}',
            f'{self.velocity_linear[0]:.4f}', f'{self.velocity_linear[1]:.4f}',
            f'{self.velocity_linear[2]:.4f}',
            f'{self.orientation_euler[0]:.6f}',
            f'{self.orientation_euler[1]:.6f}',
            f'{self.orientation_euler[2]:.6f}',
            f'{self.velocity_angular[0]:.6f}',
            f'{self.velocity_angular[1]:.6f}',
            f'{self.velocity_angular[2]:.6f}',
            f'{self.accel_linear[0]:.4f}', f'{self.accel_linear[1]:.4f}',
            f'{self.accel_linear[2]:.4f}',
            f'{self.motor_commands[0]:.4f}', f'{self.motor_commands[1]:.4f}',
            f'{self.motor_commands[2]:.4f}', f'{self.motor_commands[3]:.4f}',
            f'{total_thrust:.4f}',
            f'{estimated_power:.2f}',
            f'{stability:.6f}',
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(f'Flight data saved to {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
