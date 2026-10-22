#!/usr/bin/env python3
"""
Command-Line Teleop for Morphing Drone
========================================
Type commands with distances for precise control.

Commands:
    FLIGHT:
        takeoff [alt]       Take off to altitude (default 5m)
        land                Land at current XY position
        w [dist]            Move forward (default 1m)
        s [dist]            Move backward
        a [dist]            Move left
        d [dist]            Move right
        up [dist]           Ascend (default 1m)
        down [dist]         Descend
        yaw [degrees]       Rotate yaw (positive = left)
        goto [x] [y] [z]   Fly to exact coordinates
        home                Return to origin at current altitude

    MORPHING:
        x                   Morph to X-config (balanced)
        o                   Morph to O-config (compact)
        y                   Morph to Y-config (cruise)
        t                   Morph to T-config (inspect)
        h                   Morph to H-config (stable landing)

    SYSTEM:
        status / st         Print current drone state
        help                Show this help
        stop                Emergency stop
        quit / exit         Shutdown
"""

import math
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


HELP_TEXT = """
┌────────────────────────────────────────────────────────────┐
│              MORPH DRONE — COMMAND TELEOP                  │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  FLIGHT:                    MORPHING:                      │
│  ──────────────────         ─────────                      │
│  takeoff [alt]              x = X-config (balanced)        │
│  land                       o = O-config (compact)         │
│  w [dist]  = forward        y = Y-config (cruise)          │
│  s [dist]  = backward       t = T-config (inspect)         │
│  a [dist]  = left           h = H-config (landing)         │
│  d [dist]  = right                                         │
│  up [dist] = ascend         SYSTEM:                        │
│  down [dist] = descend      ──────                         │
│  yaw [deg] = rotate         status = print state           │
│  goto [x] [y] [z]          stop   = emergency stop        │
│  home = return to 0,0       help   = show commands         │
│                             quit   = shutdown              │
│                                                            │
│  Examples:                                                 │
│    w 5        → move forward 5 meters                      │
│    up 3       → ascend 3 meters                            │
│    goto 5 0 8 → fly to position (5, 0, 8)                  │
│    t          → morph to T-config                          │
│    takeoff 10 → take off to 10m altitude                   │
│                                                            │
└────────────────────────────────────────────────────────────┘
"""

CONFIGS = {'x': 'X', 'o': 'O', 'y': 'Y', 't': 'T', 'h': 'H'}


def quaternion_to_euler(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class TeleopController(Node):
    def __init__(self):
        super().__init__('teleop_controller')

        # Publishers
        self.config_pub = self.create_publisher(
            String, '/morphing_drone/set_config', 10)
        self.flight_enable_pub = self.create_publisher(
            Bool, '/morphing_drone/flight_enabled', 10)
        self.target_pub = self.create_publisher(
            Float64MultiArray, '/morphing_drone/target_position', 10)
        self.yaw_pub = self.create_publisher(
            Float64, '/morphing_drone/target_yaw', 10)

        # Subscribers
        self.create_subscription(
            Odometry, '/morphing_drone/ground_truth/odom',
            self._odom_callback, 10)
        self.create_subscription(
            String, '/morphing_drone/current_config',
            self._config_callback, 10)

        # State
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0]
        self.current_config = "X"
        self.is_flying = False

        # Target
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0

        # Spin ROS in a background thread so input() doesn't block callbacks
        self.spin_thread = threading.Thread(target=self._spin_background, daemon=True)
        self.spin_thread.start()

    def _spin_background(self):
        """Run ROS spin in background so we get odometry updates while waiting for input."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _odom_callback(self, msg):
        self.position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        self.velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]
        r, p, y = quaternion_to_euler(msg.pose.pose.orientation)
        self.orientation = [math.degrees(r), math.degrees(p), math.degrees(y)]

    def _config_callback(self, msg):
        self.current_config = msg.data

    def _publish_target(self):
        msg = Float64MultiArray()
        msg.data = [self.target_x, self.target_y, self.target_z]
        self.target_pub.publish(msg)

    def _publish_yaw(self):
        msg = Float64()
        msg.data = math.radians(self.target_yaw)
        self.yaw_pub.publish(msg)

    def _set_flight(self, enabled):
        msg = Bool()
        msg.data = enabled
        self.flight_enable_pub.publish(msg)
        self.is_flying = enabled

    def _set_config(self, config_name):
        msg = String()
        msg.data = config_name
        self.config_pub.publish(msg)

    def _print_status(self):
        status = "FLYING" if self.is_flying else "GROUNDED"
        dist = math.sqrt(
            (self.position[0] - self.target_x)**2 +
            (self.position[1] - self.target_y)**2 +
            (self.position[2] - self.target_z)**2
        )
        print(f"""
┌─ DRONE STATUS ───────────────────────────────────────┐
│  State:    {status:<12}  Config:  {self.current_config}                   │
│  Position: x={self.position[0]:+7.2f}  y={self.position[1]:+7.2f}  z={self.position[2]:6.2f}m     │
│  Target:   x={self.target_x:+7.2f}  y={self.target_y:+7.2f}  z={self.target_z:6.2f}m     │
│  Distance to target: {dist:.2f}m                            │
│  Attitude: roll={self.orientation[0]:+5.1f}° pitch={self.orientation[1]:+5.1f}° yaw={self.orientation[2]:+6.1f}°│
│  Velocity: vx={self.velocity[0]:+5.2f}  vy={self.velocity[1]:+5.2f}  vz={self.velocity[2]:+5.2f} m/s  │
│  Target yaw: {self.target_yaw:+.0f}°                                     │
└──────────────────────────────────────────────────────┘""")

    def _parse_float(self, parts, index, default):
        """Safely parse a float from command parts."""
        if index < len(parts):
            try:
                return float(parts[index])
            except ValueError:
                print(f"  ✗ Invalid number: '{parts[index]}'")
                return None
        return default

    def process_command(self, raw):
        """Parse and execute a single command."""
        raw = raw.strip()
        if not raw:
            return True

        parts = raw.lower().split()
        cmd = parts[0]

        # ─── TAKEOFF ───
        if cmd == 'takeoff':
            alt = self._parse_float(parts, 1, 5.0)
            if alt is None:
                return True
            alt = max(1.0, min(50.0, alt))
            self.target_z = alt
            self._set_flight(True)
            self._publish_target()
            print(f"  ✓ Taking off to {alt:.1f}m")

        # ─── LAND ───
        elif cmd == 'land':
            print(f"  ✓ Landing at ({self.target_x:.1f}, {self.target_y:.1f})...")
            self.target_z = 0.3
            self._publish_target()
            # Let it descend, then disable flight
            import time
            time.sleep(4.0)
            self._set_flight(False)
            self.target_z = 0.0
            print(f"  ✓ Landed — flight disabled")

        # ─── MOVEMENT ───
        elif cmd == 'd':
            dist = self._parse_float(parts, 1, 1.0)
            if dist is None:
                return True
            self.target_x += dist
            self._publish_target()
            print(f"  ✓ Forward {dist:.1f}m → target ({self.target_x:+.1f}, {self.target_y:+.1f}, {self.target_z:.1f})")

        elif cmd == 'a':
            dist = self._parse_float(parts, 1, 1.0)
            if dist is None:
                return True
            self.target_x -= dist
            self._publish_target()
            print(f"  ✓ Backward {dist:.1f}m → target ({self.target_x:+.1f}, {self.target_y:+.1f}, {self.target_z:.1f})")

        elif cmd == 'w':
            dist = self._parse_float(parts, 1, 1.0)
            if dist is None:
                return True
            self.target_y += dist
            self._publish_target()
            print(f"  ✓ Left {dist:.1f}m → target ({self.target_x:+.1f}, {self.target_y:+.1f}, {self.target_z:.1f})")

        elif cmd == 's':
            dist = self._parse_float(parts, 1, 1.0)
            if dist is None:
                return True
            self.target_y -= dist
            self._publish_target()
            print(f"  ✓ Right {dist:.1f}m → target ({self.target_x:+.1f}, {self.target_y:+.1f}, {self.target_z:.1f})")

        elif cmd == 'up':
            dist = self._parse_float(parts, 1, 1.0)
            if dist is None:
                return True
            self.target_z += dist
            self._publish_target()
            print(f"  ✓ Up {dist:.1f}m → altitude {self.target_z:.1f}m")

        elif cmd == 'down':
            dist = self._parse_float(parts, 1, 1.0)
            if dist is None:
                return True
            self.target_z = max(0.5, self.target_z - dist)
            self._publish_target()
            print(f"  ✓ Down {dist:.1f}m → altitude {self.target_z:.1f}m")

        elif cmd == 'yaw':
            deg = self._parse_float(parts, 1, 15.0)
            if deg is None:
                return True
            self.target_yaw += deg
            if self.target_yaw > 180:
                self.target_yaw -= 360
            elif self.target_yaw < -180:
                self.target_yaw += 360
            self._publish_yaw()
            print(f"  ✓ Yaw {deg:+.0f}° → heading {self.target_yaw:.0f}°")

        elif cmd == 'goto':
            if len(parts) < 4:
                print("  ✗ Usage: goto [x] [y] [z]   Example: goto 5 0 8")
                return True
            gx = self._parse_float(parts, 1, None)
            gy = self._parse_float(parts, 2, None)
            gz = self._parse_float(parts, 3, None)
            if gx is None or gy is None or gz is None:
                return True
            self.target_x = gx
            self.target_y = gy
            self.target_z = max(0.5, gz)
            self._publish_target()
            dist = math.sqrt(
                (self.position[0] - gx)**2 +
                (self.position[1] - gy)**2 +
                (self.position[2] - gz)**2
            )
            print(f"  ✓ Going to ({gx:.1f}, {gy:.1f}, {gz:.1f}) — {dist:.1f}m away")

        elif cmd == 'home':
            self.target_x = 0.0
            self.target_y = 0.0
            self.target_yaw = 0.0
            self._publish_target()
            self._publish_yaw()
            print(f"  ✓ Returning to origin at altitude {self.target_z:.1f}m")

        # ─── MORPHING ───
        elif cmd in CONFIGS:
            config_name = CONFIGS[cmd]
            self._set_config(config_name)
            print(f"  ✓ Morphing to {config_name}-config")

        # ─── STATUS ───
        elif cmd in ('status', 'st'):
            self._print_status()

        # ─── EMERGENCY STOP ───
        elif cmd == 'stop':
            self._set_flight(False)
            self.is_flying = False
            self.target_z = 0.0
            print("  ⚠ EMERGENCY STOP — flight disabled")

        # ─── HELP ───
        elif cmd == 'help':
            print(HELP_TEXT)

        # ─── QUIT ───
        elif cmd in ('quit', 'exit'):
            if self.is_flying:
                print("  Landing first...")
                self._set_flight(False)
            print("  Shutting down.")
            return False

        # ─── UNKNOWN ───
        else:
            print(f"  ✗ Unknown command: '{cmd}' — type 'help' for commands")

        return True  # continue running

    def run(self):
        """Main command loop."""
        print(HELP_TEXT)
        print("  Waiting for Gazebo connection...")
        print("  Type 'takeoff' to start, 'help' for commands\n")

        running = True
        while running and rclpy.ok():
            try:
                # Build prompt with current state
                config = self.current_config
                alt = self.position[2]
                state = "FLY" if self.is_flying else "GND"
                prompt = f"  [{state}|{config}|{alt:.1f}m] > "

                raw = input(prompt)
                running = self.process_command(raw)
            except (KeyboardInterrupt, EOFError):
                print("\n  Shutting down...")
                if self.is_flying:
                    self._set_flight(False)
                running = False


def main(args=None):
    rclpy.init(args=args)
    node = TeleopController()
    try:
        node.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
