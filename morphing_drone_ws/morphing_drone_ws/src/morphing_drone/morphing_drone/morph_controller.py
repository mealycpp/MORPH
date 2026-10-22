#!/usr/bin/env python3
"""
Morph Controller — Commands servo joints to change drone shape.

Listens for configuration change requests on /morphing_drone/set_config
and smoothly transitions servo joints to the target angles.

Publishes current servo angles for the flight controller's dynamic mixer.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# Configuration definitions: servo deflection angles in degrees
# These deflect the OUTER arm segment relative to the inner arm's direction.
# Arms numbered 1-4: FR(45°), FL(135°), RL(225°), RR(315°) in X-config.
#
# Angles derived from physical prototype photos:
#   O: all arms sweep inward ~90° so EDFs cluster into a compact circle
#   T: all 4 arms move (front spreads, rear folds inward) — extreme asymmetric
#   Y: rear pair folds inward — mild asymmetric
#   H: front pair sweeps forward, rear pair sweeps backward → parallel H bars
CONFIGURATIONS = {
    'X': {
        'description': 'Standard X quadcopter — all arms straight',
        'servo_angles_deg': [0.0, 0.0, 0.0, 0.0],
    },
    'O': {
        'description': 'Circular/compact — all arms sweep +90°',
        'servo_angles_deg': [90.0, 90.0, 90.0, 90.0],
    },
    'T': {
        'description': 'T-shape — all 4 arms move',
        'servo_angles_deg': [-45.0, 45.0, 45.0, -45.0],
    },
    'Y': {
        'description': 'Y-shape — rear pair folds, front stays',
        'servo_angles_deg': [0.0, 0.0, 45.0, -45.0],
    },
    'H': {
        'description': 'H-shape — front pair forward, rear pair backward',
        'servo_angles_deg': [45.0, -45.0, 45.0, -45.0],
    },
}


class MorphController(Node):

    def __init__(self):
        super().__init__('morph_controller')

        self.declare_parameter('transition_time_s', 5.0)
        self.transition_time = self.get_parameter('transition_time_s').value

        self.current_config = 'X'
        self.current_angles_rad = np.zeros(4)
        self.target_angles_rad = np.zeros(4)
        self.transitioning = False
        self.transition_start_time = None
        self.transition_start_angles = np.zeros(4)

        self.joint_names = [
            'servo_joint_1', 'servo_joint_2',
            'servo_joint_3', 'servo_joint_4'
        ]

        # Publisher: joint trajectory for Gazebo joint controller
        # Gazebo's libgazebo_ros_joint_pose_trajectory listens on set_joint_trajectory
        self.joint_traj_pub = self.create_publisher(
            JointTrajectory,
            '/morphing_drone/set_joint_trajectory',
            10
        )

        # Publisher: current servo angles (for flight controller mixer)
        self.servo_angles_pub = self.create_publisher(
            Float64MultiArray,
            '/morphing_drone/servo_angles',
            10
        )

        # Publisher: current configuration name
        self.config_pub = self.create_publisher(
            String,
            '/morphing_drone/current_config',
            10
        )

        # Subscriber: receive config change commands
        self.create_subscription(
            String,
            '/morphing_drone/set_config',
            self._set_config_callback,
            10
        )

        # Timer for smooth transition updates
        self.timer = self.create_timer(0.02, self._update)  # 50 Hz

        self.get_logger().info('Morph controller ready — starting in X-config')
        self._publish_config_name()

    def _set_config_callback(self, msg):
        config_name = msg.data.upper()
        if config_name not in CONFIGURATIONS:
            self.get_logger().warn(
                f'Unknown config "{config_name}". '
                f'Valid: {list(CONFIGURATIONS.keys())}'
            )
            return

        if config_name == self.current_config and not self.transitioning:
            self.get_logger().info(f'Already in {config_name} config')
            return

        target_deg = CONFIGURATIONS[config_name]['servo_angles_deg']
        self.target_angles_rad = np.radians(target_deg)
        self.transition_start_angles = self.current_angles_rad.copy()
        self.transition_start_time = self.get_clock().now().nanoseconds * 1e-9
        self.transitioning = True
        self.current_config = config_name

        self.get_logger().info(
            f'Morphing to {config_name}: '
            f'{CONFIGURATIONS[config_name]["description"]}'
        )

    def _update(self):
        t_now = self.get_clock().now().nanoseconds * 1e-9

        if self.transitioning:
            elapsed = t_now - self.transition_start_time
            alpha = min(elapsed / self.transition_time, 1.0)

            # Smooth S-curve interpolation (sinusoidal ease in/out)
            alpha_smooth = 0.5 * (1.0 - math.cos(math.pi * alpha))

            self.current_angles_rad = (
                self.transition_start_angles
                + alpha_smooth * (self.target_angles_rad - self.transition_start_angles)
            )

            if alpha >= 1.0:
                self.transitioning = False
                self.current_angles_rad = self.target_angles_rad.copy()
                self.get_logger().info(
                    f'Morph to {self.current_config} complete'
                )

            # Send joint trajectory command
            self._publish_joint_trajectory()

        # Always publish current angles and config
        angles_msg = Float64MultiArray()
        angles_msg.data = self.current_angles_rad.tolist()
        self.servo_angles_pub.publish(angles_msg)
        self._publish_config_name()

    def _publish_joint_trajectory(self):
        traj = JointTrajectory()
        # frame_id must reference a link in the model so the plugin
        # can find the correct model to actuate
        traj.header.frame_id = 'base_link'
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.current_angles_rad.tolist()
        point.velocities = [0.0] * 4
        point.time_from_start = Duration(sec=0, nanosec=50_000_000)  # 50ms

        traj.points.append(point)
        self.joint_traj_pub.publish(traj)

    def _publish_config_name(self):
        msg = String()
        msg.data = self.current_config
        self.config_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MorphController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
