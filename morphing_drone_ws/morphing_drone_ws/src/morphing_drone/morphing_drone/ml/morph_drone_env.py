"""
MorphDroneEnv - Gymnasium environment for RL training.
Standalone physics sim (no Gazebo needed), runs 1000x faster than real-time.
"""
import math
import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:
    import gym
    from gym import spaces

G = 9.80665
ARM_INNER = 0.159
ARM_OUTER = 0.159
BASE_HEADINGS_RAD = np.array([45, 135, 225, 315]) * np.pi / 180

CONFIGS = {
    'X': np.array([0.0, 0.0, 0.0, 0.0]),
    'O': np.array([90.0, 90.0, 90.0, 90.0]),
    'T': np.array([-45.0, 45.0, 45.0, -45.0]),
    'Y': np.array([0.0, 0.0, 45.0, -45.0]),
    'H': np.array([45.0, -45.0, 45.0, -45.0]),
}
CONFIG_NAMES = list(CONFIGS.keys())


def compute_edf_positions(servo_angles_deg):
    positions = np.zeros((4, 2))
    for i in range(4):
        bh = BASE_HEADINGS_RAD[i]
        sx = ARM_INNER * math.cos(bh)
        sy = ARM_INNER * math.sin(bh)
        oh = bh + math.radians(servo_angles_deg[i])
        positions[i, 0] = sx + ARM_OUTER * math.cos(oh)
        positions[i, 1] = sy + ARM_OUTER * math.sin(oh)
    return positions


class MorphDroneEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self, render_mode=None, **kwargs):
        super().__init__()
        self.mass = kwargs.get("mass", 2.5)
        self.max_thrust_per_edf = kwargs.get("max_thrust", 17.86)
        self.servo_speed = kwargs.get("servo_speed", 300.0)
        self.target_altitude = kwargs.get("target_alt", 5.0)
        self.dt = kwargs.get("dt", 0.005)
        self.max_episode_steps = kwargs.get("max_steps", 4000)
        self.spin_dirs = np.array([1, -1, 1, -1], dtype=np.float64)
        self.torque_coeff = 0.01
        self.linear_drag = 0.3
        self.angular_drag = 0.5

        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(18,), dtype=np.float32)
        self.action_space = spaces.Box(
            low=np.array([0, 0, 0, 0, 0.1, -1], dtype=np.float32),
            high=np.array([1, 1, 1, 1, 3.0, 1], dtype=np.float32),
        )
        self.render_mode = render_mode
        self.reset()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if options and "from_config" in options:
            self.from_config = options["from_config"]
            self.to_config = options["to_config"]
        else:
            idxs = self.np_random.choice(len(CONFIG_NAMES), size=2, replace=False)
            self.from_config = CONFIG_NAMES[idxs[0]]
            self.to_config = CONFIG_NAMES[idxs[1]]

        self.from_angles = CONFIGS[self.from_config].copy()
        self.to_angles = CONFIGS[self.to_config].copy()
        self.servo_angles = self.from_angles.copy()
        self.pos = np.array([0.0, 0.0, self.target_altitude])
        self.vel = np.zeros(3)
        self.orient = np.zeros(3)
        self.ang_vel = np.zeros(3)
        self.orient += self.np_random.normal(0, 0.02, 3)
        self.vel += self.np_random.normal(0, 0.05, 3)
        self.morph_progress = 0.0
        self.morph_active = False
        self.morph_start_step = 0
        self.morph_speed = 1.0
        self.step_count = 0
        self.pre_morph_steps = 100
        self.prev_action = np.zeros(6)
        self._update_dynamics()
        return self._get_obs(), {}

    def _update_dynamics(self):
        self.edf_positions = compute_edf_positions(self.servo_angles)
        m = self.mass / 4.0
        self.inertia = np.array([
            np.sum(m * self.edf_positions[:, 1] ** 2),
            np.sum(m * self.edf_positions[:, 0] ** 2),
            np.sum(m * np.sum(self.edf_positions ** 2, axis=1)),
        ])
        self.cg_offset = np.mean(self.edf_positions, axis=0)

    def _get_obs(self):
        pos_error = self.pos - np.array([0, 0, self.target_altitude])
        servo_norm = self.servo_angles / 90.0
        mt = min(1.0, (self.step_count - self.morph_start_step) * self.dt / 5.0) if self.morph_active else 0.0
        return np.concatenate([pos_error, self.vel, self.orient, self.ang_vel, servo_norm, [self.morph_progress], [mt]]).astype(np.float32)

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        motor_cmds = action[0:4] * self.max_thrust_per_edf
        morph_speed_mult = action[4]
        alt_bias = action[5] * 2.0

        if self.step_count >= self.pre_morph_steps and not self.morph_active:
            self.morph_active = True
            self.morph_start_step = self.step_count

        if self.morph_active and self.morph_progress < 1.0:
            self.morph_speed = morph_speed_mult
            max_delta = self.servo_speed * morph_speed_mult * self.dt
            delta = self.to_angles - self.servo_angles
            self.servo_angles += np.clip(delta, -max_delta, max_delta)
            total_travel = np.max(np.abs(self.to_angles - self.from_angles))
            if total_travel > 0:
                self.morph_progress = min(1.0, np.max(np.abs(self.servo_angles - self.from_angles)) / total_travel)
            else:
                self.morph_progress = 1.0
            self._update_dynamics()

        roll, pitch, yaw = self.orient
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        total_thrust = np.sum(motor_cmds) + alt_bias
        thrust_world = np.array([
            total_thrust * (sr * sy + cr * cy * sp),
            total_thrust * (-sr * cy + cr * sy * sp),
            total_thrust * cr * cp,
        ])
        gravity = np.array([0, 0, -self.mass * G])
        drag = -self.linear_drag * self.vel * np.abs(self.vel)

        torque = np.zeros(3)
        for i in range(4):
            torque[0] += motor_cmds[i] * self.edf_positions[i, 1]
            torque[1] -= motor_cmds[i] * self.edf_positions[i, 0]
            torque[2] += self.spin_dirs[i] * motor_cmds[i] * self.torque_coeff
        torque[0] -= self.mass * G * self.cg_offset[1]
        torque[1] += self.mass * G * self.cg_offset[0]
        torque += -self.angular_drag * self.ang_vel * np.abs(self.ang_vel)

        accel = (thrust_world + gravity + drag) / self.mass
        self.vel += accel * self.dt
        self.pos += self.vel * self.dt
        ang_accel = torque / np.maximum(self.inertia, 1e-6)
        self.ang_vel += ang_accel * self.dt
        self.orient += self.ang_vel * self.dt
        self.orient[2] = (self.orient[2] + math.pi) % (2 * math.pi) - math.pi
        self.step_count += 1

        alt_error = abs(self.pos[2] - self.target_altitude)
        pos_xy_error = math.sqrt(self.pos[0] ** 2 + self.pos[1] ** 2)
        attitude_error = math.sqrt(self.orient[0] ** 2 + self.orient[1] ** 2)

        reward = (
            -5.0 * alt_error ** 2
            - 20.0 * attitude_error ** 2
            - 2.0 * pos_xy_error ** 2
            - 1.0 * np.linalg.norm(self.ang_vel) ** 2
            - 0.5 * np.sum((action[0:4] - self.prev_action[0:4]) ** 2)
            + 0.1
        )
        if self.morph_progress >= 0.99:
            reward += 5.0
        if self.morph_active and attitude_error < 0.1:
            reward += 0.05 * morph_speed_mult

        self.prev_action = action.copy()
        terminated = False
        truncated = self.step_count >= self.max_episode_steps

        if self.pos[2] < 0.3:
            terminated, reward = True, reward - 100.0
        if attitude_error > math.radians(60):
            terminated, reward = True, reward - 50.0
        if pos_xy_error > 10.0:
            terminated, reward = True, reward - 50.0

        info = {
            "altitude": self.pos[2],
            "attitude_error_deg": math.degrees(attitude_error),
            "morph_progress": self.morph_progress,
            "config_from": self.from_config,
            "config_to": self.to_config,
            "servo_angles": self.servo_angles.copy(),
            "morph_speed": self.morph_speed,
        }
        return self._get_obs(), reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "human":
            att = math.degrees(math.sqrt(self.orient[0]**2 + self.orient[1]**2))
            print(f"Step {self.step_count:4d} | Alt: {self.pos[2]:5.2f}m | Att: {att:4.1f}deg | Morph: {self.morph_progress:.0%} ({self.from_config}->{self.to_config})")
