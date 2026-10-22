#!/usr/bin/env python3
"""
Obstacle Course Mission Runner
-------------------------------
Autonomous waypoint mission that demonstrates each morph configuration's
real-world advantage:

  ZONE 1 — X-config open waypoint run (pylon weaving)
  ZONE 2 — O-config narrow tunnel   (compact, fits tight spaces)
  ZONE 3 — Y-config gate slalom      (forward cruise efficiency)
  ZONE 4 — T-config wall inspection  (asymmetric body offset toward surface)
  ZONE 5 — H-config narrow slot + landing pad (slim profile + wide stance)

Publishes:
  /morphing_drone/target_position  (Float64MultiArray [x, y, z])
  /morphing_drone/set_config       (String — config name)
  /morphing_drone/flight_enabled   (Bool)

The flight controller / bridge subscribes to target_position and applies
a position-hold force toward the waypoint.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64MultiArray


# ── Mission: list of (config, x, y, z, yaw_deg, hold_seconds, description) ──
# Yaw convention: 0° = drone's front (+X body) points along world +X
#                 90° = front points along world +Y (drone's "left" side faces +X world)
#                 -90° = front points along world -Y (drone's "right" side faces +X world)
# The drone maintains a consistent body-front throughout — yaw rotates the whole drone
# in the world frame, not its internal axes.
#
# For the wall-approach (T-zone), the drone yaws so its body's -Y side (where T's
# asymmetric arms physically extend) faces the wall. That puts T's "sensor side"
# against the surface while keeping the drone's front facing +Y world (left of flight path).
MISSION = [
    # TAKEOFF — yaw 0, front faces +X (direction of course travel)
    ('X', 0.0,  0.0, 3.0,   0, 6.0,  'Takeoff — climb to 3m'),

    # ZONE 1: X-config open waypoints (drone faces +X throughout)
    ('X', 5.0, -3.0, 3.5,   0, 5.0,  'Zone 1 — fly around pylon 1'),
    ('X', 8.0,  3.0, 4.5,   0, 5.0,  'Zone 1 — fly around pylon 2 (climb)'),
    ('X', 12.0, -2.0, 4.0,  0, 5.0,  'Zone 1 — fly around pylon 3'),
    ('X', 14.0,  0.0, 3.0,  0, 4.0,  'Zone 1 — exit, descend to tunnel height'),

    # ZONE 2: O-config tunnel (compact all directions, no yaw needed)
    ('O', 15.0,  0.0, 3.0,  0, 5.0,  'Zone 2 — morph to O, align with tunnel'),
    ('O', 19.5,  0.0, 3.0,  0, 6.0,  'Zone 2 — fly through compact tunnel'),
    ('O', 21.0,  0.0, 3.0,  0, 3.0,  'Zone 2 — tunnel exit'),

    # ZONE 3: Y-config gate slalom — drone keeps front facing +X (flight direction).
    # Y's asymmetric arms are visible on the drone's side, but the drone flies front-first
    # with the GPS/sensor heading consistent throughout. The CG-centering math handles
    # the asymmetric thrust requirement automatically.
    ('Y', 22.0,  0.0, 2.0,  0, 4.0,  'Zone 3 — morph to Y, descend to gate height'),
    ('Y', 24.5,  0.0, 2.0,  0, 3.0,  'Zone 3 — through gate 1'),
    ('Y', 27.5,  0.0, 2.0,  0, 3.0,  'Zone 3 — through gate 2'),
    ('Y', 30.5,  0.0, 2.0,  0, 3.0,  'Zone 3 — through gate 3'),
    ('Y', 32.0,  0.0, 2.5,  0, 3.0,  'Zone 3 — exit slalom'),

    # ZONE 4: T-config wall inspection
    # NOTE: True wall-side approach would require yawing the drone, but our current
    # bridge can't reliably stabilize during yaw transitions in extreme asymmetric configs.
    # For the demo, we approach the wall in the standard heading; T's extreme CG offset
    # is still showcased via the visible asymmetric morph and per-motor thrust split.
    ('T', 33.0,  0.0, 2.0,  0, 5.0, 'Zone 4 — morph to T, approach wall'),
    ('T', 34.6,  0.0, 2.0,  0, 3.0, 'Zone 4 — close approach'),
    ('T', 34.7,  0.0, 2.0,  0, 5.0, 'Zone 4 — HOLD — taking sensor reading'),
    ('T', 33.0,  0.0, 2.0,  0, 3.0, 'Zone 4 — retreat from wall'),

    # ZONE 5: H-config slot + landing — back to normal yaw, front faces +X
    ('H', 40.0,  0.0, 2.4,  0, 5.0,  'Zone 5 — morph to H, align with slot'),
    ('H', 42.0,  0.0, 2.4,  0, 4.0,  'Zone 5 — through slot'),
    ('H', 44.0,  0.0, 2.4,  0, 3.0,  'Zone 5 — clear of slot'),
    ('H', 46.0,  0.0, 2.0,  0, 4.0,  'Zone 5 — over landing pad (stable hover)'),
    ('H', 46.0,  0.0, 0.4,  0, 6.0,  'Zone 5 — DESCEND and land'),
]

# Waypoint tolerance before advancing (meters)
WAYPOINT_TOL = 0.6
# Max time to wait for a waypoint to be reached (s)
WAYPOINT_TIMEOUT = 25.0

# Ground-phase morph demo duration per shape (shorter than before)
GROUND_SEQUENCE = [
    ('X', 5.0),
    ('O', 5.0),
    ('Y', 5.0),
    ('T', 5.0),
    ('H', 5.0),
    ('X', 5.0),
]


class ObstacleCourseMission(Node):

    def __init__(self):
        super().__init__('obstacle_course_mission')

        self.config_pub = self.create_publisher(
            String, '/morphing_drone/set_config', 10)
        self.flight_enable_pub = self.create_publisher(
            Bool, '/morphing_drone/flight_enabled', 10)
        self.target_pub = self.create_publisher(
            Float64MultiArray, '/morphing_drone/target_position', 10)

        from std_msgs.msg import Float64
        self.yaw_pub = self.create_publisher(
            Float64, '/morphing_drone/target_yaw', 10)

        # Subscribe to odom to know when we reached waypoints
        from nav_msgs.msg import Odometry
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.create_subscription(
            Odometry, '/morphing_drone/ground_truth/odom',
            self._odom_callback, 10)

        self.get_logger().info('Obstacle course mission node initialized.')

    def _odom_callback(self, msg):
        p = msg.pose.pose.position
        self.pos_x = p.x
        self.pos_y = p.y
        self.pos_z = p.z

    def _publish_config(self, name):
        msg = String()
        msg.data = name
        self.config_pub.publish(msg)

    def _publish_target(self, x, y, z):
        msg = Float64MultiArray()
        msg.data = [float(x), float(y), float(z)]
        self.target_pub.publish(msg)

    def _publish_yaw(self, yaw_deg):
        from std_msgs.msg import Float64
        msg = Float64()
        msg.data = math.radians(yaw_deg)
        self.yaw_pub.publish(msg)

    def _distance_to(self, x, y, z):
        return math.sqrt((x - self.pos_x)**2 + (y - self.pos_y)**2 + (z - self.pos_z)**2)

    def _wait_seconds(self, seconds):
        """Block for `seconds` of wall-clock time."""
        start = self.get_clock().now().nanoseconds * 1e-9
        while (self.get_clock().now().nanoseconds * 1e-9 - start) < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_until_reached(self, x, y, z, tol=WAYPOINT_TOL, timeout=WAYPOINT_TIMEOUT):
        """Wait until the drone is within tol of the target, or timeout elapses."""
        start = self.get_clock().now().nanoseconds * 1e-9
        while rclpy.ok():
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - start
            d = self._distance_to(x, y, z)
            if d < tol:
                return True
            if elapsed > timeout:
                self.get_logger().warn(
                    f'Waypoint timeout after {timeout}s, distance still {d:.2f}m — continuing')
                return False
            # Re-publish target every ~0.1s in case bridge missed it
            self._publish_target(x, y, z)
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def run_mission(self):
        # Initial warm-up — let all the other nodes connect
        self.get_logger().info('Waiting 3s for all publishers/subscribers to connect...')
        self._wait_seconds(3.0)

        self.get_logger().info('═' * 60)
        self.get_logger().info('PHASE 1 — GROUND MORPHING DEMO')
        self.get_logger().info('═' * 60)

        flight_off = Bool(); flight_off.data = False
        self.flight_enable_pub.publish(flight_off)

        for config, duration in GROUND_SEQUENCE:
            self.get_logger().info(f'[GROUND] Morphing to {config} for {duration}s')
            self._publish_config(config)
            self._wait_seconds(duration)

        self.get_logger().info('═' * 60)
        self.get_logger().info('PHASE 2 — OBSTACLE COURSE')
        self.get_logger().info('═' * 60)

        # Enable flight — drone takes off via altitude controller
        flight_on = Bool(); flight_on.data = True
        self.flight_enable_pub.publish(flight_on)
        self._publish_config('X')

        # Publish initial takeoff target
        self._publish_target(0.0, 0.0, 3.0)
        self._wait_seconds(2.0)  # let altitude controller engage

        for i, (config, x, y, z, yaw_deg, hold, desc) in enumerate(MISSION):
            self.get_logger().info(
                f'\n─── Waypoint {i+1}/{len(MISSION)} [{config}] → ({x:.1f}, {y:.1f}, {z:.1f}) yaw={yaw_deg}° ───')
            self.get_logger().info(f'    {desc}')

            # Send new config FIRST if it changed
            self._publish_config(config)

            # Command target position and yaw
            self._publish_target(x, y, z)
            self._publish_yaw(yaw_deg)

            # Wait for arrival (or timeout)
            reached = self._wait_until_reached(x, y, z)
            status = 'REACHED' if reached else 'TIMEOUT'
            self.get_logger().info(
                f'    {status} — current pos: ({self.pos_x:.2f}, {self.pos_y:.2f}, {self.pos_z:.2f})')

            # Hold at this waypoint for the specified duration
            self.get_logger().info(f'    Holding for {hold}s...')
            self._wait_seconds(hold)

        self.get_logger().info('═' * 60)
        self.get_logger().info('MISSION COMPLETE — landing sequence')
        self.get_logger().info('═' * 60)

        # Disable flight (kills thrust, drone settles)
        self.flight_enable_pub.publish(flight_off)
        self._wait_seconds(3.0)

        self.get_logger().info('Shutting down obstacle course node.')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleCourseMission()
    try:
        node.run_mission()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
