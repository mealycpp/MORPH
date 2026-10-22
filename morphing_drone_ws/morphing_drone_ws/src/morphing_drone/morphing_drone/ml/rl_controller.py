#!/usr/bin/env python3
"""
RL Policy Runner - deploys trained model as a ROS 2 node.
Hybrid mode: PID handles hover, RL takes over during morph transitions.
"""
import os, math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String

MODEL_PATH = os.path.expanduser("~/morphing_drone_data/ml/best_model.zip")

try:
    from stable_baselines3 import PPO
    ML_AVAILABLE = True
except ImportError:
    ML_AVAILABLE = False

CONFIGS = {
    'X': np.array([0.0, 0.0, 0.0, 0.0]),
    'O': np.array([90.0, 90.0, 90.0, 90.0]),
    'T': np.array([-45.0, 45.0, 45.0, -45.0]),
    'Y': np.array([0.0, 0.0, 45.0, -45.0]),
    'H': np.array([45.0, -45.0, 45.0, -45.0]),
}

def quat_to_euler(q):
    x, y, z, w = q
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(np.clip(2*(w*y-z*x), -1, 1))
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return roll, pitch, yaw

class RLController(Node):
    def __init__(self):
        super().__init__('rl_controller')
        self.declare_parameter('mode', 'hybrid')
        self.declare_parameter('target_altitude', 5.0)
        self.declare_parameter('model_path', MODEL_PATH)

        self.mode = self.get_parameter('mode').value
        self.target_alt = self.get_parameter('target_altitude').value
        path = self.get_parameter('model_path').value

        if not ML_AVAILABLE:
            self.get_logger().error("stable-baselines3 not installed"); return
        if not os.path.exists(path):
            self.get_logger().error(f"No model at {path}. Train first."); return

        self.model = PPO.load(path)
        self.get_logger().info(f"RL model loaded. Mode: {self.mode}")

        self.pos = np.zeros(3); self.vel = np.zeros(3)
        self.orient = np.zeros(3); self.ang_vel = np.zeros(3)
        self.servo_angles = np.zeros(4)
        self.current_config = 'X'; self.target_config = 'X'
        self.morph_active = False; self.morph_progress = 0.0
        self.morph_start_time = None; self.state_ready = False
        self.max_thrust = 17.86

        self.thrust_pubs = [self.create_publisher(Wrench, f'/morphing_drone/edf_{i}/thrust_cmd', 10) for i in range(1,5)]
        self.morph_speed_pub = self.create_publisher(Float64MultiArray, '/morphing_drone/rl_morph_speed', 10)

        self.create_subscription(Odometry, '/morphing_drone/ground_truth/odom', self._odom_cb, 10)
        self.create_subscription(Imu, '/morphing_drone/imu/data', self._imu_cb, 10)
        self.create_subscription(Float64MultiArray, '/morphing_drone/servo_angles', self._servo_cb, 10)
        self.create_subscription(String, '/morphing_drone/current_config', self._config_cb, 10)
        self.create_subscription(String, '/morphing_drone/set_config', self._target_cb, 10)

        self.timer = self.create_timer(0.005, self._control_loop)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position; self.pos = np.array([p.x, p.y, p.z])
        v = msg.twist.twist.linear; self.vel = np.array([v.x, v.y, v.z])
        q = msg.pose.pose.orientation; self.orient = np.array(quat_to_euler((q.x, q.y, q.z, q.w)))
        self.state_ready = True

    def _imu_cb(self, msg):
        av = msg.angular_velocity; self.ang_vel = np.array([av.x, av.y, av.z])

    def _servo_cb(self, msg):
        if len(msg.data) == 4: self.servo_angles = np.array(msg.data)

    def _config_cb(self, msg): self.current_config = msg.data

    def _target_cb(self, msg):
        t = msg.data.upper()
        if t != self.current_config and t in CONFIGS:
            self.target_config = t; self.morph_active = True
            self.morph_start_time = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(f"RL morph: {self.current_config} -> {t}")

    def _control_loop(self):
        if not self.state_ready or not hasattr(self, 'model'): return
        if self.mode == 'hybrid' and not self.morph_active: return

        pos_err = self.pos - np.array([0, 0, self.target_alt])
        s_norm = np.degrees(self.servo_angles) / 90.0
        mt = min(1.0, (self.get_clock().now().nanoseconds*1e-9 - self.morph_start_time)/5.0) if self.morph_active and self.morph_start_time else 0.0
        obs = np.concatenate([pos_err, self.vel, self.orient, self.ang_vel, s_norm, [self.morph_progress], [mt]]).astype(np.float32)

        action, _ = self.model.predict(obs, deterministic=True)
        for i in range(4):
            w = Wrench(); w.force.z = float(action[i] * self.max_thrust)
            self.thrust_pubs[i].publish(w)

        msg = Float64MultiArray(); msg.data = [float(action[4])]
        self.morph_speed_pub.publish(msg)

        if self.morph_progress >= 0.99 and self.morph_active:
            self.morph_active = False; self.current_config = self.target_config
            self.get_logger().info(f"Morph complete -> {self.current_config}")

def main(args=None):
    rclpy.init(args=args)
    node = RLController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
