#!/usr/bin/env python3
"""
Gazebo Drone Controller Bridge
-------------------------------
Publishes wrench to /morphing_drone/gazebo_ros_force.

CRITICAL: libgazebo_ros_force applies forces in WORLD frame.
So we must rotate the body-frame thrust vector by the drone's
current orientation before publishing.

Body frame: thrust is along +Z (up through the EDF ducts)
World frame: we rotate this by the drone's roll/pitch/yaw
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench, Vector3
from nav_msgs.msg import Odometry

ARM_INNER = 0.159
ARM_OUTER = 0.159
BASE_HEADINGS_RAD = np.array([45, 135, 225, 315]) * np.pi / 180


def compute_edf_positions(servo_angles_rad):
    positions = np.zeros((4, 2))
    for i in range(4):
        bh = BASE_HEADINGS_RAD[i]
        sx = ARM_INNER * math.cos(bh)
        sy = ARM_INNER * math.sin(bh)
        oh = bh + servo_angles_rad[i]
        positions[i, 0] = sx + ARM_OUTER * math.cos(oh)
        positions[i, 1] = sy + ARM_OUTER * math.sin(oh)
    return positions


def quat_to_euler(qx, qy, qz, qw):
    roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    pitch = math.asin(np.clip(2*(qw*qy - qz*qx), -1, 1))
    yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return roll, pitch, yaw


class GazeboDroneBridge(Node):

    def __init__(self):
        super().__init__('gazebo_drone_bridge')

        self.spin_dirs = np.array([1.0, -1.0, 1.0, -1.0])
        self.torque_coeff = 0.01
        self.servo_angles_rad = np.zeros(4)
        self.motor_thrusts = np.zeros(4)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.ang_vel_x = 0.0
        self.ang_vel_y = 0.0
        self.ang_vel_z = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        # Waypoint target (updated by obstacle course runner)
        self.target_x = 0.0
        self.target_y = 0.0
        self.has_waypoint = False
        # Yaw target (radians, world frame). 0 = drone's +X axis points along world +X
        self.target_yaw = 0.0

        # Publish to force plugin topic
        self.wrench_pub = self.create_publisher(
            Wrench, '/morphing_drone/gazebo_ros_force', 10
        )

        # Subscribe to motor commands
        self.create_subscription(
            Float64MultiArray, '/morphing_drone/motor_commands',
            self._motor_cmd_callback, 10
        )
        self.create_subscription(
            Float64MultiArray, '/morphing_drone/servo_angles',
            self._servo_callback, 10
        )
        # Subscribe to odometry to get current orientation
        self.create_subscription(
            Odometry, '/morphing_drone/ground_truth/odom',
            self._odom_callback, 10
        )
        # Subscribe to waypoint target (obstacle course mode)
        self.create_subscription(
            Float64MultiArray, '/morphing_drone/target_position',
            self._target_callback, 10
        )
        # Subscribe to target yaw (radians, world frame)
        from std_msgs.msg import Float64
        self.create_subscription(
            Float64, '/morphing_drone/target_yaw',
            self._yaw_callback, 10
        )

        self.timer = self.create_timer(0.01, self._publish_wrench)
        self.get_logger().info(
            'Gazebo drone bridge started — body-frame thrust rotation enabled'
        )

    def _motor_cmd_callback(self, msg):
        if len(msg.data) >= 4:
            self.motor_thrusts = np.array(msg.data[:4])

    def _servo_callback(self, msg):
        if len(msg.data) == 4:
            self.servo_angles_rad = np.array(msg.data)

    def _target_callback(self, msg):
        if len(msg.data) >= 2:
            self.target_x = msg.data[0]
            self.target_y = msg.data[1]
            self.has_waypoint = True

    def _yaw_callback(self, msg):
        self.target_yaw = float(msg.data)

    def _odom_callback(self, msg):
        p = msg.pose.pose.position
        self.pos_x = p.x
        self.pos_y = p.y
        q = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        av = msg.twist.twist.angular
        self.ang_vel_x = av.x
        self.ang_vel_y = av.y
        self.ang_vel_z = av.z
        v = msg.twist.twist.linear
        self.vel_x = v.x
        self.vel_y = v.y

    def _publish_wrench(self):
        total_thrust = float(np.sum(self.motor_thrusts))

        if total_thrust < 0.01:
            wrench = Wrench()
            self.wrench_pub.publish(wrench)
            return

        edf_pos = compute_edf_positions(self.servo_angles_rad)

        # Compute torques from differential thrust
        roll_torque = 0.0
        pitch_torque = 0.0
        yaw_torque = 0.0
        for i in range(4):
            t = self.motor_thrusts[i]
            roll_torque += t * edf_pos[i, 1]
            pitch_torque -= t * edf_pos[i, 0]
            yaw_torque += self.spin_dirs[i] * t * self.torque_coeff

        # CG feedforward: cancels constant moment from thrust centroid offset
        # Only applies when CG is actually offset (Y, T configs).
        # X, O, H are symmetric so cx=cy=0 → no contribution.
        cx = float(np.mean(edf_pos[:, 0]))
        cy = float(np.mean(edf_pos[:, 1]))
        cg_offset_mm = math.sqrt(cx*cx + cy*cy) * 1000

        if cg_offset_mm > 1.0:  # Only compensate when offset > 1mm (Y, T)
            cg_gain = 0.65
            roll_torque -= cg_gain * total_thrust * cy
            pitch_torque -= cg_gain * (-total_thrust * cx)

        # Body-frame thrust is [0, 0, total_thrust] (up through EDFs)
        # Rotate to world frame using current orientation
        cr = math.cos(self.roll)
        sr = math.sin(self.roll)
        cp = math.cos(self.pitch)
        sp = math.sin(self.pitch)
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)

        # Rotation matrix: body Z-axis thrust to world frame
        fx_world = total_thrust * (sr*sy + cr*cy*sp)
        fy_world = total_thrust * (-sr*cy + cr*sy*sp)
        fz_world = total_thrust * (cr*cp)

        # Velocity damping: gently oppose horizontal drift so it doesn't
        # accumulate across shapes. Small gain — just enough to bleed off
        # drift between transitions, not enough to fight intentional motion.
        vel_kd = 1.5  # N per m/s
        max_damp = 2.0  # N
        damp_fx = float(np.clip(-vel_kd * self.vel_x, -max_damp, max_damp))
        damp_fy = float(np.clip(-vel_kd * self.vel_y, -max_damp, max_damp))

        # Waypoint tracking: apply horizontal force toward target position
        # Only active when a waypoint has been set (obstacle course mode)
        wp_fx = 0.0
        wp_fy = 0.0
        if self.has_waypoint:
            err_x = self.target_x - self.pos_x
            err_y = self.target_y - self.pos_y
            pos_kp = 2.5  # N per meter of position error
            max_wp_force = 6.0  # N
            wp_fx = float(np.clip(pos_kp * err_x, -max_wp_force, max_wp_force))
            wp_fy = float(np.clip(pos_kp * err_y, -max_wp_force, max_wp_force))

        # Attitude stabilization: apply corrective torques to keep drone level
        att_kp = 2.0   # N*m per radian of tilt
        att_kd = 0.5   # N*m per rad/s of angular velocity

        stab_roll_torque = -att_kp * self.roll - att_kd * self.ang_vel_x
        stab_pitch_torque = -att_kp * self.pitch - att_kd * self.ang_vel_y
        # Yaw control — track target heading (world frame)
        # Shortest-angle error, wrapped to [-pi, pi]
        yaw_err = self.target_yaw - self.yaw
        while yaw_err > math.pi: yaw_err -= 2 * math.pi
        while yaw_err < -math.pi: yaw_err += 2 * math.pi
        stab_yaw_torque = 0.8 * yaw_err - 0.3 * self.ang_vel_z

        wrench = Wrench()
        wrench.force = Vector3(
            x=float(fx_world + damp_fx + wp_fx),
            y=float(fy_world + damp_fy + wp_fy),
            z=float(fz_world)
        )
        wrench.torque = Vector3(
            x=float(roll_torque + stab_roll_torque),
            y=float(pitch_torque + stab_pitch_torque),
            z=float(yaw_torque + stab_yaw_torque)
        )
        self.wrench_pub.publish(wrench)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboDroneBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
