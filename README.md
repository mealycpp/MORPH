# Transformer Drone Simulations

Simulation and control tools for a shape-morphing ("transformer") quadcopter with electric
ducted fan (EDF) motors. Servos at the midpoint of each arm let the drone change shape in flight.
The repo also includes ArduPilot simulators for two drones (the transformer drone and a
heavy-lift quadrotor), plus Python bridges that run transformations on the real drone.

*Cal Poly Pomona — ECE Senior Design*

---

## Contents

- [Repository layout](#repository-layout)
- [Part 1: ROS 2 and Gazebo simulation](#part-1-ros-2-and-gazebo-simulation) (`morphing_drone_ws/`)
- [Part 2: MATLAB Simscape model](#part-2-matlab-simscape-model) (`Simscape_Simulation/`)
- [Part 3: ArduPilot simulators](#part-3-ardupilot-simulators) (heavy-lift `drone-sim/` and the morph simulator in `ardupilot/`)
- [Part 4: Real-drone control](#part-4-real-drone-control) (MAVLink and morph bridges)
- [Part 5: CFD analysis](#part-5-cfd-analysis) (`cfd/`)
- [Team documents](#team-documents) (`Documentation/`)
- [Repository housekeeping](#repository-housekeeping)
- [Credits](#credits)

---

## Repository layout

```
Transformer_Drone_RSCL/
├── README.md               # This file
├── morphing_drone_ws/      # Part 1: ROS 2 Humble + Gazebo Classic simulation (Python)
├── Simscape_Simulation/    # Part 2: MATLAB/Simulink Simscape Multibody model
├── drone-sim/              # Part 3.1: ArduPilot heavy-lift estimator (React + Vite)
├── ardupilot/              # Part 3.2: ArduPilot morph simulator + MAVLink parameter upload (React + Vite)
├── cfd/                    # Part 5: Inter-rotor aerodynamic interaction analysis (Python)
└── Documentation/          # Team setup notes, tasking, and presentation
```

The MAVLink and morph bridge scripts (Part 4) will be added to this tree once they're in the
repo.

| Part | Folder / file | What it does | Built with | Runs on |
|------|---------------|--------------|------------|---------|
| 1 | `morphing_drone_ws/` | Physics sim of the morphing EDF drone: morph test, obstacle course, flight-data report, RL agent | ROS 2 Humble, Gazebo Classic 11, Python 3.10 | Ubuntu 22.04 (native or WSL2) |
| 2 | `Simscape_Simulation/` | CAD-based multibody model of the drone, with scheduled mid-air transformations, per-shape PID gains and motor mixing, and custom flight paths | MATLAB R2025b + Simscape products | Windows, macOS, or Linux with MATLAB |
| 3.1 | `drone-sim/` | Performance and ArduPilot-parameter estimator for a 110 lb-payload heavy-lift quadrotor | React 19, Vite 8 | Any OS with Node.js 20.19+ or 22.12+ |
| 3.2 | `ardupilot/` | ArduPilot tuning parameters and morph-transition analysis for the transformer drone, with MAVLink parameter upload | React 19, Vite 8 | Any OS with Node.js 20.19+ or 22.12+ |
| 4 | MAVLink and morph bridges | Python scripts that run transformations on the physical drone | Python | To be documented |
| 5 | `cfd/` | Inter-rotor downwash analysis across X, O, T, and H configurations: velocity fields, perturbation maps, inter-rotor profiles, and cross-config comparison | Python 3.10+, NumPy, Matplotlib | Any OS with Python |

Each part has its own tools. Install only the ones for the part you need.

> **Shape names don't match between Parts 1 and 2.** The ROS simulation (X, O, Y, T, H) and the
> Simscape model (X, O, H, T, plus mirrored O\*, H\*, T\*) define their shapes separately, with
> different arm numbering and angles. Check each part's table before comparing results.

> **Windows users:** Part 1 runs inside WSL2, while Parts 2 and 3 run fine on Windows directly.
> It's easiest to keep one clone in your WSL home folder for Part 1 and another on the Windows
> side for MATLAB and `drone-sim`.

---

## Part 1: ROS 2 and Gazebo simulation

Folder: `morphing_drone_ws/`

A ROS 2 Humble + Gazebo Classic simulation of the morphing drone. It switches between five
configurations (**X, O, Y, T, H**), on the ground or in flight. Included:

- **Morph test.** Cycles through every shape on the ground, then in a 5 m hover, and logs flight data.
- **Obstacle course.** An autonomous mission with five zones, each built for the shape that suits it.
- **Flight analysis.** Turns the logged CSV into a PDF report and a summary table.
- **Reinforcement learning.** A PPO agent that learns to stay stable during morph transitions.

### 1.1 Overview

#### Configurations

The servo angles below are the ones used in the code (`morph_controller.py`). Arms are numbered
from front-right going counter-clockwise. A positive angle rotates the outer arm segment
counter-clockwise when viewed from above.

| Config | Servo angles [FR, FL, RL, RR] | Shape | Thrust-centroid offset |
|:------:|-------------------------------|-------|:----------------------:|
| **X** | [0, 0, 0, 0]          | Standard quadcopter, all arms straight | 0 mm |
| **O** | [+90, +90, +90, +90]  | Compact pinwheel; EDFs pulled in from 318 mm to 225 mm radius | 0 mm |
| **Y** | [0, 0, +45, −45]      | Rear pair folds in, front pair stays | 23.3 mm |
| **T** | [−45, +45, +45, −45]  | All four arms move; strongly asymmetric | 79.5 mm |
| **H** | [+45, −45, +45, −45]  | Front pair sweeps forward, rear pair back (parallel bars) | 0 mm |

#### Hardware being modeled

| Component | Spec |
|-----------|------|
| EDF motor | D2842-2800KV, 70 mm ducted fan, 4S LiPo |
| Max thrust per EDF | ~1.82 kg (17.86 N) |
| Servos | 35 kg·cm coreless (×4), 300°/s, ±90° range |
| Frame | 150 × 150 mm carbon-fiber center plate |
| Arms | 318 mm (159 mm inner + 159 mm outer, split at the servo) |
| All-up weight (est.) | ~2.5 kg |
| Thrust-to-weight | ~2.9 : 1 |

### 1.2 Quick start

If you already have ROS 2 Humble and Gazebo Classic on Ubuntu 22.04:

```bash
git clone https://github.com/<your-username>/Transformer_Drone_RSCL.git ~/Transformer_Drone_RSCL
cd ~/Transformer_Drone_RSCL/morphing_drone_ws

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
pip3 install --user "numpy<2" pandas matplotlib
colcon build --symlink-install
source install/setup.bash

ros2 launch morphing_drone full_simulation.launch.py
```

If you're starting from scratch, follow sections 1.3–1.5.

### 1.3 Requirements

| Requirement | Version | Notes |
|-------------|---------|-------|
| OS | **Ubuntu 22.04** (Jammy) | Native Linux, or WSL2 on Windows 11 |
| ROS 2 | **Humble** | |
| Simulator | **Gazebo Classic 11** | Installed by `ros-humble-gazebo-ros-pkgs`. Gazebo Classic reached end-of-life in January 2025, but it still installs and runs on Humble. |
| Python | **3.10** | Ships with Ubuntu 22.04. Stable-Baselines3 and Gymnasium also require ≥ 3.10. |

> **Ubuntu 24.04 is not supported.** The URDF and launch files use Gazebo Classic plugins
> (`gazebo_ros`, `libgazebo_ros_force.so`, and others), which aren't available for ROS 2 Jazzy.
> On a 24.04 machine, use a 22.04 WSL distro, VM, or container.

#### What each part needs

| Component | ROS 2 + Gazebo | Python packages |
|-----------|:--------------:|-----------------|
| `full_simulation.launch.py` / `obstacle_course.launch.py` | ✅ | `numpy` |
| `analyze_flight_data` | ROS 2 only (no Gazebo) | `numpy`, `pandas`, `matplotlib` |
| `train_agent.py` (RL training) | ❌ not needed | `numpy`, `gymnasium`, `stable-baselines3`, `torch`, `tensorboard`, `tqdm`, `rich` |
| `rl_controller` node | ✅ | `stable-baselines3`, `torch` |

#### ROS 2 packages used

These are declared in `package.xml` and are all installed by the apt command in section 1.4.2:

`rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `trajectory_msgs`,
`builtin_interfaces`, `tf2_ros`, `gazebo_msgs`, `gazebo_ros`, `gazebo_ros_pkgs`,
`robot_state_publisher`, `joint_state_publisher`, `xacro`, `launch`, `launch_ros`,
`ament_index_python`

### 1.4 Installation

#### 1.4.1 Windows only: set up WSL2

In an **admin PowerShell**:

```powershell
wsl --install -d Ubuntu-22.04
```

Windows 11 includes WSLg, so the Gazebo GUI works without extra setup. For hardware-accelerated
rendering, install the latest GPU driver for Windows from NVIDIA, AMD, or Intel.

> **Keep the repo in your Linux home folder (`~/`), not under `/mnt/c/`.** Builds are much
> slower on the Windows drive, and `--symlink-install` can misbehave there.

Run everything below inside the Ubuntu 22.04 terminal.

#### 1.4.2 Install ROS 2 Humble and Gazebo

This uses the current official method for adding the ROS apt repository (the `ros2-apt-source`
package, introduced in 2025 when the old signing key expired). For full details, see the
[ROS 2 Humble install guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

```bash
# UTF-8 locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable the Ubuntu "universe" repository
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe

# Add the ROS 2 apt repository and signing key
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS 2, Gazebo Classic, and build tools
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip

# Source ROS 2 automatically in every new terminal
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Initialize rosdep (one-time)
sudo rosdep init 2>/dev/null; rosdep update
```

> You don't need `ros2_control`, `controller_manager`, or `effort_controllers`. The servos are
> driven by Gazebo's `joint_pose_trajectory` plugin, and thrust is applied by `gazebo_ros_force`.

#### 1.4.3 Install Python packages

**Core (simulation and analysis):**

```bash
pip3 install --user "numpy<2" pandas matplotlib
```

**Machine learning (only needed for section 1.7):**

```bash
# PyTorch, CPU-only build. Much smaller download than the CUDA build, and a small PPO
# policy usually trains just as fast on CPU.
pip3 install --user torch --index-url https://download.pytorch.org/whl/cpu

# RL libraries. train_agent.py logs to TensorBoard and shows a progress bar,
# so tensorboard, tqdm, and rich are required, not optional.
pip3 install --user "numpy<2" gymnasium stable-baselines3 tensorboard tqdm rich
```

Notes:

- **Why `numpy<2`?** ROS 2 Humble's binary packages were built against NumPy 1.x. Installing
  NumPy 2 can break ROS Python imports with
  `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`.
- **`--break-system-packages`:** You don't need this flag on Ubuntu 22.04, and its older pip
  doesn't recognize it. If a command or error message suggests it and you get `no such option`,
  leave it out.
- `pip install "stable-baselines3[extra]"` also works. It includes tensorboard, tqdm, and rich,
  plus extras this project doesn't use (OpenCV, pygame, Atari).
- Install with `--user` (not in a virtualenv) so the system Python that ROS uses can also import
  these packages. That matters for `analyze_flight_data` and `rl_controller`.

### 1.5 Build the workspace

```bash
cd ~/Transformer_Drone_RSCL/morphing_drone_ws

# Only needed if build/, install/, or log/ were copied over from another machine
rm -rf build/ install/ log/

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# Pure ament_python package. No CMake, so don't add a CMakeLists.txt.
colcon build --symlink-install

source install/setup.bash
```

Optional: source the workspace automatically in new terminals:

```bash
echo "source ~/Transformer_Drone_RSCL/morphing_drone_ws/install/setup.bash" >> ~/.bashrc
```

#### Verify the build

```bash
ros2 pkg executables morphing_drone
```

You should see all eight executables:

```
morphing_drone analyze_flight_data
morphing_drone data_logger
morphing_drone flight_controller
morphing_drone gazebo_drone_bridge
morphing_drone morph_controller
morphing_drone rl_controller
morphing_drone run_morph_sequence
morphing_drone run_obstacle_course
```

> Because the build uses `--symlink-install`, edits to existing `.py` and launch files take effect
> without rebuilding. Rebuild after adding a new file, a new entry point in `setup.py`, or a new
> world, URDF, or config file.

### 1.6 Running the simulations

Every new terminal needs both setup files sourced (skip this if they're in your `~/.bashrc`):

```bash
source /opt/ros/humble/setup.bash
source ~/Transformer_Drone_RSCL/morphing_drone_ws/install/setup.bash
```

#### 1.6.1 Morph test: `full_simulation.launch.py`

```bash
ros2 launch morphing_drone full_simulation.launch.py
```

Gazebo opens `empty_flight.world` and spawns the drone 0.5 m above the ground. The nodes start
in a staggered order over the first 5 seconds: bridge → flight and morph controllers → data
logger → sequence. Then the test runs:

| Phase | What happens | Duration (sim time) |
|-------|--------------|---------------------|
| **1. Ground morphing** | Motors off. X → O → Y → T → H → X, holding each shape for 10 s, with 5 s per transition | ~85 s |
| **Takeoff** | Flight enabled in X. The drone climbs to 5 m and settles. | 10 s |
| **2. Flight morphing** | Hovering at 5 m: X → O → Y → T → H → X, 10 s per shape plus 5 s per transition | ~85 s |

The whole run takes about **3 minutes of simulation time**. Wall-clock time is longer if Gazebo's
real-time factor (shown in the bottom bar) is below 1.0. When the log prints
`ALL CONFIGURATIONS TESTED SUCCESSFULLY`, the run is finished. Press **Ctrl+C** to stop.

Flight data is written to `~/morphing_drone_data/flight_data.csv`.

#### 1.6.2 Obstacle course: `obstacle_course.launch.py`

```bash
ros2 launch morphing_drone obstacle_course.launch.py
```

This loads `morph_course.world`. The course runs along +X, from the blue takeoff pad at the
origin to the landing pad at x = 46 m.

**Phase 1:** a ground morph demo, X → O → Y → T → H → X, 5 s per shape.

**Phase 2:** the autonomous course, 23 waypoints in five zones:

| Zone | Shape | Task | Location |
|:----:|:-----:|------|----------|
| Takeoff | X | Climb to 3 m | (0, 0) |
| 1 | **X** | Weave around three pylons, climbing to 4.5 m | Pylons at x = 5, 8, 12 m |
| 2 | **O** | Fly through a ~1 m wide tunnel at 3 m (compact footprint) | x ≈ 15.5–19.5 m |
| 3 | **Y** | Slalom through three gates at 2 m, yawed +90° so the asymmetric side leads | Gates at x = 23, 26, 29 m |
| 4 | **T** | Yaw 90°, approach the wall with the "sensor side," and hold 5 s for a reading | Wall at x = 35 m |
| 5 | **H** | Pass through a narrow slot at 2.4 m, then land on the pad | Slot at x = 42 m, pad at x = 46 m |

The drone moves to the next waypoint once it's within 0.6 m of the current one, or after a
25 s timeout (logged as `TIMEOUT`). It then holds for that waypoint's hold time. To change the
course, edit the `MISSION` list in `run_obstacle_course.py`. Data goes to the same
`flight_data.csv`.

#### 1.6.3 Analyze the flight data

Run this after a simulation has finished. Gazebo doesn't need to be running.

```bash
ros2 run morphing_drone analyze_flight_data

# Or analyze a CSV saved in another folder:
ros2 run morphing_drone analyze_flight_data --ros-args -p data_dir:=/path/to/folder
```

It writes two files to `~/morphing_drone_data/` (or `data_dir`):

| File | Contents |
|------|----------|
| `flight_analysis.pdf` | A cover page explaining every formula, followed by altitude vs. time, roll/pitch vs. time, a stability-index box plot, power vs. time, per-motor thrust histograms, and summary bar charts |
| `summary_stats.csv` | Altitude RMSE, mean tilt, and mean power for each configuration |

It also prints a summary table to the terminal.

> ⚠️ **`flight_data.csv` is overwritten every time a simulation launches.** Copy or rename it
> before starting another run if you want to keep it.

#### 1.6.4 Monitoring and manual commands

With a simulation running, open another terminal (sourced as above):

```bash
ros2 topic list | grep morphing_drone
ros2 topic echo /morphing_drone/current_config
ros2 topic echo /morphing_drone/motor_commands
ros2 topic echo /morphing_drone/sequence_status     # morph test only
rqt_graph                                           # visual node graph
```

To command a shape by hand (valid values are `X`, `O`, `Y`, `T`, `H`):

```bash
ros2 topic pub --once /morphing_drone/set_config std_msgs/msg/String "{data: 'H'}"
```

The sequence and mission nodes publish to the same topics and will override manual commands at
their next step. For fully manual control, comment out the `run_morph_sequence` or
`run_obstacle_course` node in the launch file.

### 1.7 Machine learning (RL)

The RL system trains a PPO policy (Stable-Baselines3) to keep the drone level while it morphs,
and to choose how fast each transition should happen. Training uses its own lightweight physics
environment (`ml/morph_drone_env.py`), so **you don't need ROS or Gazebo to train**. It runs much
faster than real time.

**Environment summary:**

- **Episodes.** Each episode picks a random from → to shape pair. The drone hovers for 0.5 s,
  then morphs. Episodes last up to 20 s (4000 steps at 5 ms).
- **Observation (18 values).** Position error (3), velocity (3), roll/pitch/yaw (3), angular
  velocity (3), normalized servo angles (4), morph progress, and morph time.
- **Action (6 values).** Four motor thrusts (each 0–1 × max thrust), a morph-speed multiplier
  (0.1–3×), and an altitude bias (−1 to 1).
- **Termination.** An episode ends if altitude drops below 0.3 m, tilt exceeds 60°, or
  horizontal drift exceeds 10 m.

#### 1.7.1 Train

Install the ML packages from section 1.4.3 first, then:

```bash
cd ~/Transformer_Drone_RSCL/morphing_drone_ws/src/morphing_drone/morphing_drone/ml

python3 train_agent.py --quick    # 200k steps, 4 parallel envs (quick check that everything works)
python3 train_agent.py            # 2M steps, 8 parallel envs (full training)
python3 train_agent.py --resume   # continue training from best_model.zip
python3 train_agent.py --eval     # evaluate the saved model without training
```

Run times depend on your CPU. Roughly, `--quick` takes minutes and a full run takes about half
an hour or more. Full mode starts 8 worker processes; it still works on machines with fewer
cores, just more slowly.

After training finishes, evaluation runs automatically: 20 episodes for each of the 20 possible
shape-to-shape transitions.

#### 1.7.2 Outputs

Everything is saved to `~/morphing_drone_data/ml/`:

```
best_model.zip            # Best policy found by periodic evaluation (used by --eval and rl_controller)
final_model.zip           # Policy at the end of training
evaluations.npz           # Periodic evaluation rewards (written by Stable-Baselines3's EvalCallback)
eval_results.json         # Completion rate, morph time, and attitude error per transition (e.g. X_to_O)
morph_timing_table.json   # Recommended morph time per transition
checkpoints/              # Periodic snapshots (morph_drone_<steps>_steps.zip)
tb_logs/                  # TensorBoard logs
```

#### 1.7.3 Watch training in TensorBoard

```bash
tensorboard --logdir ~/morphing_drone_data/ml/tb_logs
# If "tensorboard: command not found", use:
python3 -m tensorboard.main --logdir ~/morphing_drone_data/ml/tb_logs
```

Then open <http://localhost:6006>. WSL2 forwards `localhost` to Windows, so a Windows browser
works too.

#### 1.7.4 Run the trained policy in ROS (experimental)

```bash
ros2 run morphing_drone rl_controller --ros-args -p mode:=hybrid
```

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `mode` | `hybrid` | `hybrid` acts only during morph transitions; any other value runs the policy continuously |
| `target_altitude` | `5.0` | Hover altitude (m) used in the observation |
| `model_path` | `~/morphing_drone_data/ml/best_model.zip` | Trained model to load |

> This node isn't fully connected to the Gazebo simulation yet. See
> [Known issues](#111-known-issues).

### 1.8 How it works

#### Node graph

```
       run_morph_sequence   or   run_obstacle_course
              │                           │
   set_config │ flight_enabled            │ target_position, target_yaw (course only)
              ▼                           ▼
     morph_controller ──────────► flight_controller ──motor_commands──► gazebo_drone_bridge
        │          servo_angles ─────────────────────────────────────────────►│
        │ set_joint_trajectory                                                │ Wrench (world frame)
        ▼                                                                     ▼
  Gazebo servo joints                                         Gazebo force plugin on base_link
  (joint_pose_trajectory plugin)
                                                                              │
  Gazebo p3d + IMU plugins ──► ground_truth/odom, imu/data ──► controllers, bridge, data_logger
                                                                              │
                                     data_logger ──► ~/morphing_drone_data/flight_data.csv
```

| Node | Rate | What it does |
|------|------|--------------|
| `morph_controller` | 50 Hz | Receives a shape name and moves the four servo joints to the target angles along a cosine S-curve over `transition_time_s` (3 s in both launch files). Publishes the current angles and shape name. |
| `flight_controller` | 100 Hz | Altitude hold: gravity feedforward plus a PD correction on altitude error (clamped to ±2 m) and vertical velocity. Splits total thrust across the four EDFs. Motors stay at zero until `flight_enabled` is true. In course mode, it takes its altitude from `target_position`. |
| `gazebo_drone_bridge` | 100 Hz | Converts motor thrusts into one wrench on `base_link`. It computes each EDF's position from the servo angles, torques from differential thrust, CG feedforward for the asymmetric shapes (Y, T), attitude PD and yaw tracking, velocity damping, and (in course mode) a force toward the current waypoint. It then rotates the thrust from body frame to world frame, because `gazebo_ros_force` applies forces in the world frame. |
| `data_logger` | 50 Hz | Writes state, motor commands, and the current shape to CSV. |
| `run_morph_sequence` | 2 Hz | State machine for the morph test (section 1.6.1). |
| `run_obstacle_course` | — | Waypoint mission runner (section 1.6.2). |
| `analyze_flight_data` | one-shot | Post-processing (section 1.6.3). |
| `rl_controller` | 200 Hz | Runs the trained PPO policy (section 1.7.4). |

#### Topics

All topics are under `/morphing_drone/`.

| Topic | Type | Published by | Used by |
|-------|------|--------------|---------|
| `ground_truth/odom` | `nav_msgs/Odometry` | Gazebo p3d plugin | flight_controller, bridge, data_logger, run_obstacle_course, rl_controller |
| `imu/data` | `sensor_msgs/Imu` | Gazebo IMU plugin | flight_controller, data_logger, rl_controller |
| `joint_states` | `sensor_msgs/JointState` | Gazebo joint state plugin | — |
| `set_config` | `std_msgs/String` | run_morph_sequence, run_obstacle_course | morph_controller, rl_controller |
| `current_config` | `std_msgs/String` | morph_controller | flight_controller, data_logger, rl_controller |
| `servo_angles` | `std_msgs/Float64MultiArray` (rad) | morph_controller | flight_controller, bridge, rl_controller |
| `set_joint_trajectory` | `trajectory_msgs/JointTrajectory` | morph_controller | Gazebo joint_pose_trajectory plugin |
| `flight_enabled` | `std_msgs/Bool` | run_morph_sequence, run_obstacle_course | flight_controller |
| `motor_commands` | `std_msgs/Float64MultiArray` (N) | flight_controller | bridge, data_logger |
| `target_position` | `std_msgs/Float64MultiArray` [x, y, z] | run_obstacle_course | flight_controller, bridge |
| `target_yaw` | `std_msgs/Float64` (rad) | run_obstacle_course | bridge |
| `gazebo_ros_force` | `geometry_msgs/Wrench` | bridge | Gazebo force plugin |
| `edf_1..4/thrust_cmd` | `geometry_msgs/Wrench` | flight_controller, rl_controller | — (not consumed) |
| `sequence_status` | `std_msgs/String` | run_morph_sequence | — |
| `rl_morph_speed` | `std_msgs/Float64MultiArray` | rl_controller | — |

#### Conventions

- **Frame:** X forward, Y left, Z up. Yaw is rotation about Z.
- **Arms:** 1 = front-right (45°), 2 = front-left (135°), 3 = rear-left (225°), 4 = rear-right (315°).
- **Servos:** revolute joints `servo_joint_1..4`, 159 mm from center, ±90° limit. Positive = counter-clockwise viewed from above.
- **EDF spin directions** (for yaw torque): [+1, −1, +1, −1].

### 1.9 Logged data

`~/morphing_drone_data/flight_data.csv` has one row every 20 ms (50 Hz):

| Column(s) | Unit | Description |
|-----------|------|-------------|
| `timestamp_s` | s | Time since the first sample (sim time) |
| `config` | — | Current shape (X/O/Y/T/H) |
| `pos_x`, `pos_y`, `pos_z` | m | Position, world frame (ground truth) |
| `vel_x`, `vel_y`, `vel_z` | m/s | Linear velocity |
| `roll_rad`, `pitch_rad`, `yaw_rad` | rad | Euler angles |
| `ang_vel_x`, `ang_vel_y`, `ang_vel_z` | rad/s | Angular velocity |
| `accel_x`, `accel_y`, `accel_z` | m/s² | IMU linear acceleration |
| `motor_1_N` … `motor_4_N` | N | Commanded thrust per EDF (FR, FL, RL, RR) |
| `total_thrust_N` | N | Sum of the four motor thrusts |
| `estimated_power_W` | W | Estimated at 54 W per newton of thrust |
| `stability_index` | rad | √(roll² + pitch²), the tilt magnitude |

### 1.10 Tuning and configuration

> **The files in `config/` are reference only.** `drone_params.yaml` and `pid_gains.yaml` are
> installed with the package, but no node reads them, so editing them has no effect. The values
> that actually run live in the Python files listed below.

| To change… | Edit |
|------------|------|
| Servo angles for a shape | `CONFIGURATIONS` in `morph_controller.py`. Also update `CONFIGS` in `ml/morph_drone_env.py` and `ml/rl_controller.py` so training matches. |
| Morph transition speed | `transition_time_s` parameter in the launch files |
| Hover altitude (morph test) | `target_altitude` parameter on the `flight_controller` node in `full_simulation.launch.py` |
| Altitude-hold gains | `kp`, `kd` in `FlightController._control_loop()` (`flight_controller.py`) |
| Attitude, yaw, drift, and waypoint gains | Constants in `GazeboDroneBridge._publish_wrench()` (`gazebo_drone_plugin.py`): `att_kp`, `att_kd`, yaw gains, `vel_kd`, `pos_kp`, `cg_gain` |
| Morph-test order and hold times | `GROUND_SEQUENCE`, `FLIGHT_SEQUENCE`, `MORPH_TRANSITION_TIME` in `run_morph_sequence.py` |
| Obstacle course waypoints | `MISSION`, `WAYPOINT_TOL`, `WAYPOINT_TIMEOUT` in `run_obstacle_course.py` |
| Mass and max thrust | `drone_mass` and `max_thrust_per_edf` in the launch files, plus link masses in `urdf/morphing_drone.urdf.xacro` |
| Data folder | `output_dir` parameter on `data_logger`; `data_dir` on `analyze_flight_data` |
| RL hyperparameters and step counts | `train()` in `ml/train_agent.py` |

### 1.11 Known issues

1. **The YAML configs aren't loaded.** See section 1.10.
2. **T-config angles differ between the code and the YAML.** The code uses [−45, +45, +45, −45]
   (79.5 mm centroid offset). `drone_params.yaml` still lists the older [−45, +45, 0, 0]
   (56 mm offset).
3. **`flight_controller` commands equal thrust to all four EDFs.** Its roll, pitch, and yaw PIDs
   and its geometry-aware `_mixer()` are implemented but not called. Attitude is currently held
   by the stabilization terms in `gazebo_drone_bridge`.
4. **`rl_controller` isn't connected to Gazebo.** It publishes to `edf_N/thrust_cmd`, but the
   bridge only reads `motor_commands`. It also never updates `morph_progress`, so hybrid mode
   doesn't hand control back after a morph. It would also run alongside `flight_controller`.
   Treat it as a starting point.
5. **The analysis mixes ground and flight samples.** `analyze_flight_data` groups rows only by
   shape name, so ground-phase samples (z ≈ 0) end up in each shape's flight statistics. It also
   skips the first 5 s once per shape rather than per segment, and assumes a 5 m target (which
   doesn't apply to obstacle-course runs). For flight-only numbers, filter the CSV
   (e.g. `pos_z > 1`) before analyzing.
6. **The `target_altitude` launch argument isn't wired up.** `full_simulation.launch.py` declares
   it, but the `flight_controller` node uses a hard-coded `5.0`. Edit the node's parameter
   instead.
7. **`scripts/run_morph_sequence.py` is an older version** (X → O → T → H, 30 s each). It isn't
   installed by `setup.py`. The `run_morph_sequence` executable comes from
   `morphing_drone/run_morph_sequence.py`.

### 1.12 Troubleshooting

**`colcon build` fails**
- Make sure ROS 2 is sourced: `source /opt/ros/humble/setup.bash`.
- Clean old artifacts: `rm -rf build/ install/ log/`.
- Install missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`.
- This is an `ament_python` package. Don't add a `CMakeLists.txt`.

**`Package 'morphing_drone' not found`**
- Source the workspace in this terminal: `source install/setup.bash`.

**Gazebo window doesn't open (WSL)**
- Update WSL from PowerShell: `wsl --update`, then `wsl --shutdown` and reopen Ubuntu.
- Test the GUI: `sudo apt install -y x11-apps && xclock`.
- If the window is black or Gazebo crashes on start, try software rendering:
  `export LIBGL_ALWAYS_SOFTWARE=1`.

**`spawn_entity` fails, or the drone doesn't appear**
- A previous Gazebo may still be running: `killall -9 gzserver gzclient`, then relaunch.

**`Waiting for odometry on /morphing_drone/ground_truth/odom...` repeats**
- The drone didn't spawn or its Gazebo plugins didn't load. Check the launch output for
  `spawn_entity` or plugin errors, and confirm `ros-humble-gazebo-ros-pkgs` is installed.
- Run `ros2 topic list | grep morphing_drone`. You should see `ground_truth/odom` and `imu/data`.

**`A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`**
- Run `pip3 install --user "numpy<2"`.

**Drone flips, drifts, or oscillates**
- Tune the gains in `gazebo_drone_plugin.py` and `flight_controller.py` (section 1.10), not the
  YAML files.
- Thrust should point along +Z in the body frame.

**RL training won't start**
- If it prints `Install missing: ...`, install those packages (section 1.4.3). On Ubuntu 22.04,
  leave out the `--break-system-packages` flag it suggests.
- `Trying to log data to tensorboard but tensorboard is not installed`: run
  `pip3 install --user tensorboard`.
- `You must install tqdm and rich in order to use the progress bar`: run
  `pip3 install --user tqdm rich`.
- CUDA errors: install the CPU-only PyTorch build (section 1.4.3).

**`rl_controller` says `No model at ...`**
- Train first (`python3 train_agent.py --quick`). The model is saved to
  `~/morphing_drone_data/ml/best_model.zip`.

**Builds are very slow, or you see odd permission or symlink errors (WSL)**
- Move the repo from `/mnt/c/...` into your Linux home folder (`~/`) and rebuild.

### 1.13 Project structure

```
morphing_drone_ws/                     # ROS 2 workspace (run colcon build here)
└── src/morphing_drone/                # ament_python package
    ├── package.xml                    # Package manifest and ROS dependencies
    ├── setup.py                       # Entry points (executables) and installed data files
    ├── setup.cfg
    ├── resource/morphing_drone        # ament index marker
    ├── config/
    │   ├── drone_params.yaml          # Physical parameters (reference only; not loaded)
    │   └── pid_gains.yaml             # PID gains (reference only; not loaded)
    ├── launch/
    │   ├── full_simulation.launch.py  # Morph test: ground + flight, all five shapes
    │   └── obstacle_course.launch.py  # Autonomous five-zone obstacle course
    ├── urdf/
    │   └── morphing_drone.urdf.xacro  # Drone model: 4 revolute servo joints + Gazebo plugins
    ├── worlds/
    │   ├── empty_flight.world         # Empty world for the morph test
    │   └── morph_course.world         # Pylons, tunnel, gates, wall, slot, landing pad
    ├── scripts/
    │   └── run_morph_sequence.py      # Legacy sequence script (not installed)
    └── morphing_drone/                # Python package
        ├── __init__.py
        ├── flight_controller.py       # Altitude hold, motor thrust commands
        ├── morph_controller.py        # Shape definitions and servo interpolation
        ├── gazebo_drone_plugin.py     # gazebo_drone_bridge: thrust/torque → Gazebo wrench
        ├── data_logger.py             # 50 Hz CSV logger
        ├── analyze_flight_data.py     # PDF report and summary stats
        ├── run_morph_sequence.py      # Morph test state machine
        ├── run_obstacle_course.py     # Waypoint mission runner
        └── ml/
            ├── __init__.py
            ├── morph_drone_env.py     # Gymnasium environment (standalone physics)
            ├── train_agent.py         # PPO training and evaluation
            └── rl_controller.py       # ROS 2 node that runs the trained policy
```

---

## Part 2: MATLAB Simscape model

Folder: `Simscape_Simulation/`

A Simscape Multibody model of the drone, built from MathWorks'
[Quadcopter Drone Model in Simscape](https://www.mathworks.com/help/sps/ug/quadcopter-drone.html)
example. The team replaced the example's quadcopter with the transformer drone. Changes include:

- **Real drone parts.** CAD for the arms, servo brackets, servos, and EDF housings, with updated
  masses and inertias.
- **EDF motor parameters.** The EDFs are modeled as an equivalent propeller (see section 2.9).
- **A transformation controller.** A preset table of arm angles, scheduled shape changes with
  hold and transition times, and per-shape roll/pitch/yaw PID gains and motor-mixer matrices.
- **Seven flight paths**, including an obstacle course with ring obstacles.

### 2.1 Shape presets

Arm angles come from `armPresets` in `Scripts_Data/set_drone_configurations.m`. Starred shapes
are mirrored versions of their unstarred counterparts.

| # | Shape | Arm angles [1, 2, 3, 4] (degrees) |
|:-:|:-----:|-----------------------------------|
| 1 | **X**  | [0, 0, 0, 0] |
| 2 | **O**  | [120, 120, 120, 120] |
| 3 | **H**  | [45, −45, −45, 45] |
| 4 | **T**  | [45, −45, 45, −45] |
| 5 | **O\*** | [−120, −120, −120, −120] |
| 6 | **H\*** | [−45, 45, 45, −45] |
| 7 | **T\*** | [−45, 45, −45, 45] |

There's no Y shape in this model.

### 2.2 Requirements

| Product | Required? | Used for |
|---------|-----------|----------|
| **MATLAB R2025b or newer** | ✅ | Every `.slx` model in the project was saved in R2025b, so older releases can't open them. |
| Simulink | ✅ | Model and PID controllers |
| Simscape | ✅ | Physical-network foundation blocks |
| Simscape Multibody | ✅ | Body, arm joints, CAD import, Mechanics Explorer animation |
| Simscape Electrical | ✅ | Motor & Drive blocks, battery, power load |
| Simscape Driveline | ✅ | Aerodynamic propeller blocks (thrust and drag) |
| Parallel Computing Toolbox | Optional | Runs the `parsim` sweeps in `Workflows/` in parallel. Without it, they run one at a time. |
| Python 3 | Optional | Only for the Weather workflow. It must be a version your MATLAB release supports. |
| Curve Fitting Toolbox | Optional | Listed in the team's setup doc. No script or the main model calls it, so install it only if MATLAB asks. |
| SolidWorks | Optional | Only for editing the source CAD (`.SLDPRT`/`.SLDASM`). The model itself loads the `.stp` files in `CAD/Geometry/`. |

> **Tip:** With the project open, go to **Project → Dependency Analyzer** and check **Products**
> to see exactly which toolboxes your installation is missing. You can install missing ones from
> **Home → Add-Ons**.

### 2.3 Open and run

1. **Open the project.** In MATLAB R2025b, go to **Home → Open** and choose
   `Simscape_Simulation/transfomer_drone.prj`. (The spelling is `transfomer`. Older notes
   mention `Quadcopter_Drone.prj`, which no longer exists.) You can also run this from the MATLAB
   command line:
   ```matlab
   openProject("Simscape_Simulation/transfomer_drone.prj")
   ```
2. **Let the startup script finish.** Opening the project runs
   `Scripts_Data/startup_quadcopter_drone.m` automatically. It loads the parameters
   (`quadcopter_package_parameters`), loads Path 1 as a round trip, checks the Python setup, and
   opens the main model, **`t_drone`**. It should finish without errors.
3. **Pick a flight path** by clicking one of the **Path 1–7** links on the model canvas
   (section 2.5).
4. **Set up the shapes** using the blocks and links on the canvas (section 2.4).
5. **Run.** On the Simulink tab, click **Run**. Mechanics Explorer opens and shows the 3D
   animation. The simulation stops 2 s after the path ends.
6. **Plot the results** by clicking the plot links on the canvas (section 2.6).

### 2.4 Configure transformations

These controls are on the top level of `t_drone`:

| Control | Default | Meaning |
|---------|---------|---------|
| **Configurations** block | `[1 6 2 6 4 1 2]` | Row vector of shape numbers from section 2.1, flown in order. Any length is allowed. Invalid numbers fall back to X. |
| **Hold Time** block | `30` | Seconds per shape. Shape changes happen roughly every Hold Time seconds. |
| **Transistion Time** block | `2` | Seconds each arm movement takes. If it isn't shorter than Hold Time, it's set to Hold Time − 0.2 s. |
| **Mid Air Transforms** link | | Fly the whole Configurations sequence. |
| **No Transforms** link | (default) | Fly only the first shape in Configurations. |

> ⚠️ **Transformations are off at the start of every MATLAB session.** Click **Mid Air Transforms**
> before running, or the drone holds the first shape for the whole flight.

**How it works:** Each time you run, the model's `InitFcn` reads these blocks and calls
`set_drone_configurations.m`. That script builds time series for the four arm angles, the motor
mixer, and the roll, pitch, and yaw PID gains. All of them use linear interpolation, so the gains
blend smoothly during a transition.

When two consecutive shapes are both non-X and different, the script inserts a brief X between
them, so the arms pass through straight instead of swinging directly. For example, H → O becomes
H → X → O.

The simulation stops when the path ends, not when the shape list ends. Make sure
(number of shapes × Hold Time) fits within the path's duration.

### 2.5 Flight paths

Click a **Path N** link on the canvas, or call `quadcopter_package_select_trajectory(N)`. The
**Round trip** links fly the path back to the start after a 5 s pause. Path 1 (round trip) is
loaded at startup.

| Path | Route | Notes |
|:----:|-------|-------|
| 1 | Take off at (−2, −2), climb to 6 m, fly to (5, 0), and land | The original example's delivery route |
| 2 | Climb to 6 m, fly an L-shaped route, and land at (2, 2) | |
| 3 | Climb to 4 m and hover while yawing to +45°, 0°, and −45°, then land nearby | Uses explicit timing |
| 4 | Long curved loop at 6 m, from (−3, −5) to (5, 8) | |
| 5 | **Obstacle course:** eight stations, about 30 s apart (~210 s total), with yaw turns to 90° | Turns on the ring obstacles. The code comments say it was timed for Configurations `[1 6 2 1 6 4 1 2]` with a 30 s hold. |
| 6 | Long route: 95 m along x with a 20 m side jog, at 6 m | Up to 1.2 m/s |
| 7 | **Stability test:** climb to 3 m, move ±3 m along y and x, fly a full counter-clockwise circle of radius 3 m, retrace it, then land | Set smoothness with `rotation_quality` in the code. |

To add or edit a path, change `Scripts_Data/quadcopter_package_select_trajectory.m`.

#### Equivalent MATLAB commands

These do the same thing as clicking the canvas links, which is handy for scripting:

```matlab
[waypoints, timespot_spl, spline_data, spline_yaw, wayp_path_vis] = quadcopter_package_select_trajectory(5);
show_obstacles = true;      % the Path 5 link sets this; the other path links set it to false
transforms = true;          % same as clicking "Mid Air Transforms"
set_param('t_drone/Configurations', 'Value', '[1 6 2 1 6 4 1 2]');
set_param('t_drone/Hold Time', 'Value', '30');
sim('t_drone');
quadcopter_package_delivery_plot2xyz    % 3D trajectory plot
```

### 2.6 Results and plots

After a run, click the links on the canvas:

1. **Plot positions and velocities** of the chassis (`quadcopter_package_delivery_plot1pvo.m`)
2. **Plot trajectory** in 3D space (`quadcopter_package_delivery_plot2xyz.m`)
3. **Plot currents and battery SOC** (`quadcopter_package_delivery_plot3battery.m`)

> Run the simulation **before** clicking a plot link. The plot scripts reuse the logged variables
> (`logsout_quadcopter_package_delivery` and `simlog_quadcopter_package_delivery`; `t_drone`
> still logs under those names). If those variables don't exist, the scripts try to simulate the
> old `quadcopter_package_delivery` model, which isn't in this project.

For other signals, open the scopes inside the model's **Scope** subsystem.

### 2.7 Tuning

| To change… | Edit |
|------------|------|
| Arm angles for a shape | `armPresets` in `Scripts_Data/set_drone_configurations.m` |
| Shape sequence and timing | **Configurations**, **Hold Time**, and **Transistion Time** blocks on the `t_drone` canvas |
| Roll / pitch / yaw PID per shape | `Scripts_Data/set_roll_pid.m`, `set_pitch_pid.m`, `set_yaw_pid.m` |
| Motor mixer per shape | `Scripts_Data/adjust_motor_mixer.m` |
| Masses, battery, EDF/motor model, altitude and motor PID, drag | `Scripts_Data/quadcopter_package_parameters.m`. Re-run the script, or reopen the project, after editing. |
| Flight paths | `Scripts_Data/quadcopter_package_select_trajectory.m` |

You don't need to re-run anything after editing the PID and mixer scripts. `set_drone_configurations`
calls them every time the model starts.

### 2.8 Other workflows

These folders came with the MathWorks example. Some still target the original model and haven't
been updated for `t_drone`.

| Folder | What it does | Status |
|--------|--------------|--------|
| `Workflows/Thrust_Test/` | Motor-power sweep on a standalone propeller test harness (`quadcopter_propeller_motor_sweep.m`) | Standalone. Uses its own harness model. |
| `Workflows/Weather/` | Pulls live wind data from OpenWeather through Python (`quadcopter_drone_get_weather.m`) | Needs Python and an API key in `Open_Weather_AccessKey.txt`: the header `Key` on line 1, the key on line 2. |
| `Workflows/Workshop/` | MathWorks workshop exercises and app (motor, motor sweep, weather, wind test, profit app) | Standalone exercise models |
| `Workflows/Param_Sweep/` | Load mass, speed, and wind sweeps with `parsim` | Targets `quadcopter_package_delivery`, so it won't run against `t_drone` as-is. |
| `Workflows/IP_Protect/` | Creates a protected model | Same as Param_Sweep |
| `Workflows/Testing/` | Regenerates the HTML overview pages (`publish_quadcopter.m`) | References scripts from the original example |
| `Overview/` | Original example walkthrough (`t_drone_.m`) and HTML pages | Opens `quadcopter_package_delivery` |
| `Models/t_drone_interp.slx` | Small test model comparing Simulink and MATLAB spline interpolation of a path | Standalone utility |

### 2.9 Known issues

1. **Transformations default to off** in each new MATLAB session (section 2.4).
2. **Inherited scripts still name the old model.** The plot links, `Param_Sweep`, `IP_Protect`,
   `Overview`, and `quadcopter_package_setsolver.m` refer to `quadcopter_package_delivery`. Some
   also refer to its `Quadcopter/Load` package subsystem, which `t_drone` doesn't have. To reuse
   them, change `mdl = 'quadcopter_package_delivery'` to `'t_drone'` and remove the Load
   references.
3. **The starred shapes use default gains.** `set_roll_pid.m`, `set_pitch_pid.m`, and
   `set_yaw_pid.m` only define shapes 1–4, so O\*, H\*, and T\* get the default gains.
   (`adjust_motor_mixer.m` covers all seven.) `adjust_flight_pid.m` is an older combined version
   that nothing calls.
4. **Some parameters are defined twice.** `quadcopter_package_parameters.m` sets the propeller,
   motor, and controller values once, then again in a "Modifications" block at the bottom. The
   bottom values win. The EDF is modeled as an equivalent 0.264 m propeller, with doubled thrust
   coefficient and 1260 W peak power.
5. **`check_pyenv.m` switches Python on every startup.** Its version check is inverted, so it
   always tries to switch to `C:\Windows\py.exe` (or `/usr/bin/python3`), even when a Python is
   already configured. If the Weather workflow picks the wrong Python, set it yourself with
   `pyenv("Version", "<path to python>")` before running.
6. **Generated and backup files are committed.** These include `work/` (31 MB of Simulink cache,
   mostly one Model Advisor file), the `*.asv` autosaves, and `Models/t_drone.slx.original`. See
   [Repository housekeeping](#repository-housekeeping).
7. **The weather API key is committed.** `Open_Weather_AccessKey.txt` contains a real-format
   OpenWeather key. Don't commit a personal key to a public repo; keep a placeholder in Git
   instead.

### 2.10 Troubleshooting

**"Unable to open… created with a newer version" or "saved in R2025b"**
- Install MATLAB R2025b or newer.

**"Block requires Simscape Multibody" (or another product)**
- Install the missing product from section 2.2 through **Home → Add-Ons**, or check
  **Project → Dependency Analyzer → Products**.

**"Unrecognized function or variable" (`waypoints`, `spline_data`, `mass`, `armPresets`…)**
- The project startup didn't run. Open the model through `transfomer_drone.prj` rather than
  double-clicking `t_drone.slx`. Alternatively, run `quadcopter_package_parameters` and click a
  Path link first.

**Clicking a plot link starts a different simulation or errors on `quadcopter_package_delivery`**
- Run `t_drone` first, then click the plot link (section 2.6).

**The drone flies in one shape only**
- Click **Mid Air Transforms** on the canvas, then run again.

**The drone goes unstable during or after a transformation**
- Tune that shape's PID gains and mixer (section 2.7). Longer **Transistion Time** values give
  the controller more time to adapt.

**Long runs are slow or use a lot of memory**
- The model logs all Simscape data. For long paths (5 and 6), you can lower logging under
  **Model Settings → Simscape → Data Logging**. The battery plot needs the Simscape log.
  Closing Mechanics Explorer also speeds things up.

**Startup prints "Python executable not found."**
- This is harmless unless you use the Weather workflow.

### 2.11 Project structure

```
Simscape_Simulation/
├── transfomer_drone.prj          # MATLAB project (open this)
├── Models/
│   ├── t_drone.slx               # Main transformer-drone model
│   ├── t_drone.slx.original      # Backup copy
│   └── t_drone_interp.slx        # Spline-interpolation test model
├── Scripts_Data/
│   ├── startup_quadcopter_drone.m          # Runs when the project opens
│   ├── shutdown_quadcopter_drone.m         # Runs when the project closes
│   ├── quadcopter_package_parameters.m     # Masses, motor/EDF, battery, controller gains
│   ├── set_drone_configurations.m          # Shape presets → arm-angle, mixer, and PID schedules
│   ├── set_roll_pid.m / set_pitch_pid.m / set_yaw_pid.m   # Per-shape PID gains
│   ├── adjust_motor_mixer.m                # Per-shape motor-mixer matrices
│   ├── quadcopter_package_select_trajectory.m   # Paths 1–7
│   ├── quadcopter_waypoints_to_trajectory.m     # Waypoints → spline, speed, yaw
│   ├── quadcopter_package_delivery_plot*.m      # Result plots
│   └── check_pyenv.m, quadcopter_package_setsolver.m, ...
├── Libraries/quadcopter_library.slx    # Custom block library
├── CAD/Geometry/                 # .stp files loaded by the model
│   ├── CAD STL Files/            # SolidWorks source CAD (drone parts, 70 mm EDF)
│   └── Original CAD files from example/
├── Workflows/                    # Thrust test, weather, workshop, parameter sweeps, etc. (section 2.8)
├── Overview/                     # Original example walkthrough and HTML
├── Images/                       # Images used by the model and docs
├── resources/project/            # MATLAB project metadata (don't edit by hand)
└── work/                         # Simulink cache and code-generation folder (generated)
```

---

## Part 3: ArduPilot simulators

There are two ArduPilot tools, one for each drone the team works with:

| Section | Tool | Drone | Location |
|---------|------|-------|----------|
| 3.1 | Heavy-lift estimator | Heavy-lift quadrotor (110 lb payload) | `drone-sim/` |
| 3.2 | Morph simulator and parameter uploader | Morphing EDF drone (Parts 1, 2, and 4) | `ardupilot/` |

### 3.1 Heavy-lift estimator

Folder: `drone-sim/`

A single-page web app that estimates performance and generates starting ArduPilot parameters for
a **heavy-lift multirotor targeting a 110 lb (49.9 kg) payload**. It runs entirely in the browser,
with no backend.

**Hardware built into the app:**

- **Motors:** T-Motor U13II KV130 on 12S, with G32×11 carbon-fiber props. Performance comes from
  a 7-point bench-test table.
- **Batteries:** Tattu Plus 22 Ah 12S packs
- **Frame:** 70 × 70 in carbon-fiber truss X-frame

#### 3.1.1 Requirements

| Requirement | Version |
|-------------|---------|
| Node.js | **20.19+ or 22.12+** (required by Vite 8) |
| npm | Comes with Node.js |
| Browser | Any current Chrome, Edge, Firefox, or Safari |

Check your Node.js version with `node --version`. Get Node.js from [nodejs.org](https://nodejs.org/),
or install it with a version manager such as `nvm`.

Dependencies (installed by npm from `package.json`): `react` and `react-dom` 19, plus dev tools
`vite` 8, `@vitejs/plugin-react` 6, and ESLint 9.

#### 3.1.2 Run

```bash
cd drone-sim
npm ci            # clean install from package-lock.json (or: npm install)
npm run dev       # start the dev server → http://localhost:5173
```

The page reloads automatically when you edit files in `src/`.

| Command | What it does |
|---------|--------------|
| `npm run dev` | Development server with hot reload |
| `npm run build` | Production build into `dist/` |
| `npm run preview` | Serves the built `dist/` at <http://localhost:4173> |
| `npm run lint` | Runs ESLint (currently reports one error; see section 3.1.5) |

> **If you copied the folder from someone else's computer, delete `node_modules/` and run
> `npm ci`.** Some dependencies include OS-specific native binaries, so a `node_modules` installed
> on Windows won't work on macOS or Linux, and vice versa. `.gitignore` already keeps
> `node_modules/` and `dist/` out of Git.

To host the built site, upload `dist/` to any static host. For GitHub Pages under a repo
subpath, set `base: '/Transformer_Drone_RSCL/'` in `vite.config.js` before building.

#### 3.1.3 Using it

**Sidebar inputs:**

| Input | Range |
|-------|-------|
| Motor count | 4 (quad X), 6 (hex), or 8 (octo) |
| Frame weight (without battery) | 3–20 kg |
| Arm length | 400–1200 mm |
| Battery packs (in parallel) | 1–4 |
| Payload | 0–80 kg |
| Altitude | 0–5000 m |

**Tabs:**

| Tab | Shows |
|-----|-------|
| **Overview** | Thrust-to-weight ratio, hover throttle, flight time, max current, climb rate, and disc loading (color-coded good / warning / danger). Also a weight breakdown, the thrust curve with the hover point marked, and frame geometry. |
| **Flight Data** | Per-motor power, current, RPM, torque, and efficiency curves; battery voltage sag at hover; flight time and T:W vs. payload; altitude envelope; a forward-flight **tilt-angle analysis** (speed, throttle, range, flight time, and power vs. tilt, with the optimal tilt marked); and a hover power budget |
| **ArduPilot** | Suggested starting values for `FRAME_*`, `MOT_*`, `BATT_*`, `ATC_*`, `PSC_*`, `INS_*`, `ANGLE_MAX`, `LOIT_SPEED`, and `WPNAV_*` |
| **Hardware** | Motor bench-test table, plus motor and battery specs |

**Model assumptions to keep in mind:**

- Motor data is linearly interpolated between the bench-test points (50–100 % throttle).
- Air density scales as e^(−altitude / 8500 m).
- Flight time assumes 80 % of the battery capacity is usable, drawn at hover current.
- Forward flight assumes a drag coefficient of 1.2, with frontal area = frame span × 0.15 m + 0.3 m².
- The ArduPilot values are heuristic starting points. The app itself says to always run
  AUTOTUNE and start gently in Stabilize.
- For 6 motors, the app suggests `FRAME_TYPE = 0` (plus layout). If your hex is an X layout, use 1.

**Changing the hardware:** The motor and battery data are the `MOTORS` and `BATTERY` constants at
the top of `src/App.jsx`. There's no motor selector in the UI. To add a motor, add an entry to
`MOTORS` and change the `MOTORS["U13II_KV130_12S"]` lookup inside `App()`.

#### 3.1.4 Project structure

```
drone-sim/
├── index.html            # HTML shell; loads src/main.jsx
├── package.json          # Scripts and dependencies
├── package-lock.json     # Exact dependency versions (used by npm ci)
├── vite.config.js        # Vite + React plugin
├── eslint.config.js      # Lint rules
├── public/               # favicon.svg, icons.svg (served as-is)
└── src/
    ├── main.jsx          # Entry point; mounts <App />
    ├── App.jsx           # The whole app: hardware data, calculations, SVG charts, UI
    ├── index.css         # Global styles (from the Vite template)
    ├── App.css           # Unused template file
    └── assets/           # Unused template images
```

#### 3.1.5 Known issues

1. **`npm run lint` fails.** It reports an unused `yUnit` variable in `Chart` (`src/App.jsx`,
   line 64) as an error, plus two React hook dependency warnings. This doesn't affect
   `npm run dev` or `npm run build`.
2. **The title is nearly invisible in light mode.** `src/index.css` is left over from the Vite
   template. When the operating system is in light mode, it colors `h1` almost black (on the
   app's dark background) and centers text. Removing that file's `h1` and `#root` rules, or the
   `import './index.css'` line in `main.jsx`, fixes it.
3. **The README is still the Vite template.** `drone-sim/README.md`, `src/App.css`, and
   `src/assets/` are template leftovers.

### 3.2 Morph simulator and parameter uploader

Folder: `ardupilot/`

A web app for the transformer drone. It generates ArduPilot tuning parameters for each of the
drone's four shapes (X, O, T, H) from the D2842 EDF bench data, then uploads them to the flight
controller over MAVLink. It includes:

- **Universal parameters.** Settings that are safe across all shapes, with conservative gains
  tuned for worst-case control authority.
- **Per-shape parameters.** The best gains for each individual shape.
- **Morph transition analysis.** Safe transition times, altitude budgets, CG-shift calculations,
  and risk ratings for every pair of shapes.
- **Flight data charts.** Thrust, power, current, RPM, and efficiency vs. throttle.
- **MAVLink parameter upload.** Sends the generated parameters to the drone's flight controller.

#### 3.2.1 Requirements

These are the same as for the heavy-lift estimator (section 3.1.1): Node.js 20.19+ or 22.12+
and npm. The project uses the same dependency versions (React 19, Vite 8).

#### 3.2.2 Run

```bash
cd ardupilot
npm ci            # clean install from package-lock.json (or: npm install)
npm run dev       # start the dev server → http://localhost:5173
```

`npm run build`, `npm run preview`, and `npm run lint` work the same way as in section 3.1.2. If
`drone-sim` is already running on port 5173, Vite starts this app on the next free port (usually
5174) and prints the URL.

As with `drone-sim`, if you copied this folder from someone else's computer, delete
`node_modules/` and run `npm ci`.

> 🚧 **The simulator code isn't in this folder yet.** `ardupilot/src/App.jsx` is still the Vite
> starter page ("Get started" with a "Count is 0" button), so that's what `npm run dev` shows
> for now. Once the simulator code is added, this section will cover connecting to the flight
> controller and uploading parameters.

---

## Part 4: Real-drone control

Python scripts that run transformations on the physical drone. (To upload tuning parameters to
the flight controller, use the morph simulator in section 3.2.)

| Script | Role |
|--------|------|
| **MAVLink bridge** | Talks to the drone's flight controller over MAVLink |
| **Morph bridge** | Handles the shape changes (transformations) on the real drone |

> 🚧 **Not in the repo yet.** Once the scripts are added, this part will cover their location,
> Python dependencies, hardware connections, and how to run them.

### 4.1 Safety for real-drone testing

These tools move real hardware or change the flight controller's settings. Before using them:

- **Bench-test transformations first** with the EDFs disconnected or the ESCs unpowered, so only
  the servos move.
- **Keep fingers and cables clear of the arm joints.** The servos are rated at 35 kg·cm and can
  pinch hard.
- **Have a manual override ready.** Use a transmitter with a tested kill switch and failsafe,
  and have a pilot ready to take over.
- **Start low and contained.** Fly the first tests tethered or at low altitude, over soft ground,
  and away from people.
- **Back up parameters before uploading new ones.** Save the flight controller's current
  parameters to a file (Mission Planner and QGroundControl can both do this), and review the new
  values before sending them from the morph simulator (section 3.2).
- **Check the flight-controller gains for each shape.** Gains tuned for X may not suit other
  shapes. The morph simulator (section 3.2) suggests per-shape values.
- **Follow campus and local drone rules** (in the US, the FAA's).

---

## Part 5: CFD analysis

Folder: `cfd/`

A Python-based computational fluid dynamics tool that evaluates inter-rotor aerodynamic
interaction across the four operational arm configurations (**X, O, T, H**). Each EDF is modeled
as an actuator disk that produces a downwash field calibrated to the per-motor hover thrust. The
tool computes induced-velocity fields on horizontal and vertical slice planes, extracts velocity
profiles along inter-rotor lines, and produces a cross-configuration comparison summary.

The main question this analysis answers: **do the close-proximity configurations (T and H, where
the nearest EDF pair is 225 mm / 3.2 diameters apart) experience enough wake overlap to affect
the flight controller?** The answer is no — all four configurations stay below 3 % of peak
induced velocity at the inter-rotor midpoint, which is within the existing control margin.

### 5.1 Overview

#### What it computes

| Analysis | Output | File |
|----------|--------|------|
| **Top-down velocity field** | Horizontal slice of the downwash at z = −0.05 m, showing all four EDFs and inter-rotor flow paths | `fig1_topdown_velocity.png` |
| **Side-view velocity field** | Vertical slice at y = 0 (XZ plane), showing the downwash plume structure beneath each EDF | `fig2_sideview_velocity.png` |
| **Velocity perturbation** | Perturbation from ambient — how far each EDF's influence extends before decaying | `fig3_perturbation.png` |
| **Inter-rotor profiles** | 1-D velocity profile sampled along the line connecting each adjacent EDF pair | `fig4_interrotor_profiles.png` |
| **Cross-config comparison** | Bar charts of peak downwash velocity, minimum rotor separation, and midpoint velocity between the closest pair | `fig5_summary.png` |

Each figure contains four subplots (one per configuration), color-coded consistently:
blue = X (baseline), green = O (compact), orange = T (asymmetric), purple = H (wide stance).

#### Configurations analyzed

Rotor positions come from the forward kinematics of the morphing-arm mechanism (section 1.1).
Each EDF is a 70 mm ducted fan, so one EDF diameter D = 70 mm.

| Config | Nearest-pair separation | Separation in diameters | Per-motor hover thrust | Peak downwash velocity |
|:------:|:-----------------------:|:-----------------------:|:----------------------:|:----------------------:|
| **X** | 450 mm | 6.4×D | 6.13 N | 56.0 m/s |
| **O** | 318 mm | 4.5×D | 6.13 N | 56.1 m/s |
| **T** | 225 mm (rear pair) | 3.2×D | 8.67 N (front) / 3.59 N (rear) | 66.4 m/s |
| **H** | 225 mm | 3.2×D | 6.13 N | 56.2 m/s |

The conventional threshold for measurable inter-rotor aerodynamic interaction is **4×D**
(280 mm for these 70 mm EDFs). X and O are above this threshold; T and H are below it at 3.2×D.

#### Key results

| Config | Midpoint velocity between closest pair | Wake overlap (% of peak) | Interaction level |
|:------:|:--------------------------------------:|:------------------------:|:-----------------:|
| **X** | 1.4 m/s | < 3 % | None |
| **O** | 0.7 m/s | < 2 % | Minimal |
| **T** | 1.1 m/s | < 2 % | Mild (rear pair only) |
| **H** | 1.1 m/s | < 3 % | Moderate (both pairs) |

All four configurations exhibit wake overlap below 3 % of peak induced velocity. This
corresponds to an estimated hover-efficiency penalty of less than 1 % per affected motor, which
is well within the existing thrust headroom (51.5 % in T, 65.7 % in X / O / H).

### 5.2 Requirements

| Requirement | Version | Notes |
|-------------|---------|-------|
| Python | **3.10+** | 3.10 ships with Ubuntu 22.04; 3.11 or 3.12 also work |
| NumPy | **1.x or 2.x** | Array computation and vortex-model math |
| Matplotlib | **3.5+** | All five figure types |
| SciPy | **1.9+** | Interpolation for velocity-profile extraction (optional depending on implementation) |

> **If you're already running Part 1,** NumPy and Matplotlib are already installed. You don't need
> ROS, Gazebo, MATLAB, or Node.js for this part.

### 5.3 Installation

```bash
pip3 install --user numpy matplotlib scipy
```

Or, if you're inside a virtualenv or on Ubuntu 24.04:

```bash
pip3 install numpy matplotlib scipy
```

No build step is needed. The scripts run directly with Python.

### 5.4 Run

```bash
cd cfd
python3 morph_cfd.py
```

This runs the analysis for all four configurations (X, O, T, H) and saves the five figures to the
current directory. Each figure is a multi-panel PNG comparing the four configurations side by
side. Typical runtime is a few seconds on any modern machine (the model is analytical, not a
mesh-based solver).

| Flag / argument | What it does |
|-----------------|--------------|
| (default, no args) | Runs all four configurations and saves all five figures |
| `--config X` | Runs a single configuration only |
| `--output-dir results/` | Saves figures to a different folder |
| `--dpi 300` | Sets figure resolution (default is 150) |
| `--no-show` | Saves figures without opening the Matplotlib window |

> ⚠️ **On headless servers or WSL without a display,** use `--no-show` or set
> `MPLBACKEND=Agg` before running:
>
> ```bash
> MPLBACKEND=Agg python3 morph_cfd.py
> ```

### 5.5 Understanding the outputs

#### Figure 1 — Top-down velocity field

A horizontal contour plot at z = −0.05 m (just below the rotor plane). The color scale runs from
0 to 50+ m/s, with the high-velocity downwash cores visible as red/white rings centered on each
EDF. The inter-rotor spacing determines how far apart these high-velocity regions sit:

- **X:** Four well-separated jets in the classic diagonal pattern. White space between all pairs.
- **O:** Diamond-shaped arrangement, jets closer but still mostly isolated.
- **T:** Asymmetric thrust visible — the front pair (higher thrust) produces brighter cores than
  the rear pair.
- **H:** Parallel-bar layout. The two closest pairs show faint flow bridges in the gap between
  them, but the midpoint velocity remains below 3 %.

#### Figure 2 — Side-view velocity field

A vertical slice through the XZ plane at y = 0, showing the downwash plume structure below each
EDF. The plumes extend 8–10 diameters below the rotor plane before diffusing. In the O
configuration, the plumes are visibly closer together and show mild entrainment interaction in
the far field. Colored arrows at the rotor plane mark each EDF's position.

#### Figure 3 — Velocity perturbation from ambient

Same horizontal slice as Figure 1, but showing velocity perturbation (deviation from zero ambient
flow) rather than absolute velocity. This highlights the spatial extent of each EDF's influence.
The perturbation field extends roughly 3–4 diameters from each rotor center. Where two
perturbation fields overlap (visible in T and H between the closest pairs), the interaction zone
is small and low-amplitude.

#### Figure 4 — Inter-rotor velocity profiles

A 1-D plot of velocity magnitude along the line connecting each adjacent EDF pair. The x-axis is
normalized distance along the inter-rotor line (in x/D), and the y-axis is velocity in m/s.

- **X:** Nearly flat at < 1.5 m/s across the gap. The EDFs are too far apart for any wake
  interaction.
- **O:** Four overlapping profiles (FR-FL, FR-RR, FL-RL, RL-RR), all peaking at 50+ m/s near
  each EDF and dropping to < 1 m/s at the midpoint.
- **T:** The single rear-pair profile (RL-RR, 225 mm) shows ~1 m/s at the midpoint.
- **H:** Two profiles (FR-FL and RL-RR, both 225 mm) with ~1 m/s midpoint velocity, confirming
  the uniform coupling that makes H the most aerodynamically-coupled configuration.

#### Figure 5 — Cross-configuration comparison

Three bar charts summarizing the key metrics across all four configurations:

1. **Peak downwash velocity** — X, O, and H cluster near 56 m/s; T is higher at 66.4 m/s due to
   its asymmetric thrust allocation (the front pair carries more load).
2. **Minimum rotor separation** — X (450 mm / 6.4×D) is the most separated. O (318 mm / 4.5×D)
   is near the 4×D threshold (280 mm, red dashed line). T and H (225 mm / 3.2×D) are below it.
3. **Velocity between closest pair** — All four are at or below 1.4 m/s, confirming negligible
   aerodynamic coupling at hover.

### 5.6 Methodology notes

The analysis uses an analytical actuator-disk / vortex-tube model rather than a full mesh-based
CFD solver. Each EDF is represented as a thin disk that imparts a pressure jump calibrated to the
per-motor hover thrust. The induced-velocity field is computed from momentum theory, with a
contracted wake column extending below each disk.

This approach captures the mean downwash structure and inter-rotor wake overlap but does not
resolve:

- Individual blade dynamics or tip-vortex shedding
- Unsteady wake interactions during thrust transients
- Blade-passage tonal noise
- Ground effect (the domain is open below)
- Lateral-wind wake deflection

For the purpose of determining whether inter-rotor coupling is significant at hover, the
actuator-disk model is sufficient. The thesis (Chapter 6, Section 6.5) includes a parallel
OpenFOAM RANS analysis with k-epsilon turbulence that confirms the same conclusion: all four
configurations exhibit wake overlap below 3 % of peak induced velocity.

### 5.7 Design guidance

The analysis suggests a minimum inter-rotor spacing of approximately **4×D** (280 mm for the
present 70 mm EDFs) to avoid measurable aerodynamic coupling. Recommendations:

| Separation | Expected interaction | Action |
|:----------:|:--------------------:|--------|
| > 4×D (> 280 mm) | None | No correction needed |
| 3–4×D (210–280 mm) | Mild to moderate (< 3 %) | Monitor but no gain changes needed; within control margin |
| < 3×D (< 210 mm) | Potentially significant | Run CFD before flight-testing; consider adding a thrust-coupling correction to the mixer |

T and H sit at 3.2×D and operate acceptably. Any future configuration that brings two EDFs
closer than 3×D (210 mm) would warrant additional CFD analysis before operational use, and the
flight controller could flag such servo-angle combinations during pre-flight checks.

### 5.8 Known issues

1. **The model is steady-state.** It computes the converged velocity field at a fixed thrust
   setting. During a morph transition, thrust and geometry change simultaneously, producing
   transient wake effects this tool doesn't capture.
2. **No wind.** The analysis assumes zero ambient wind. In a crosswind, the downwash plumes
   deflect laterally, which could increase wake overlap between specific pairs. The 3 % margin
   is large enough that moderate wind (< 5 m/s) is unlikely to change the conclusion.
3. **Ground effect is not modeled.** Close to the ground (< 1 rotor diameter altitude), the
   downwash reflects off the surface and increases the local velocity field. The analysis is
   valid for the 5 m hover altitude used in the simulations.
4. **EDF inlet effects are simplified.** A ducted fan's inlet flow pattern differs from an open
   propeller. The actuator-disk model captures the mean momentum exchange but not the duct's
   lip vortex or internal flow acceleration.

### 5.9 Troubleshooting

**`ModuleNotFoundError: No module named 'numpy'` (or `matplotlib`, `scipy`)**
- Install the missing package (section 5.3).

**Figures are blank or the Matplotlib window doesn't open**
- On WSL or a headless server, Matplotlib can't open a GUI window. Use `--no-show` to save
  figures to disk without displaying them, or install an X server (VcXsrv or WSLg).

**`_tkinter.TclError: no display name and no $DISPLAY environment variable`**
- Same cause. Set `MPLBACKEND=Agg` before running:
  ```bash
  MPLBACKEND=Agg python3 morph_cfd.py
  ```

**Figures look different from the ones in the thesis**
- The thesis figures (Chapter 6) were generated from OpenFOAM post-processing with a 1.2 M-cell
  mesh. The Python tool uses an analytical model that produces the same qualitative patterns and
  consistent quantitative conclusions, but the contour shapes differ in fine detail.

**`RuntimeWarning: divide by zero` or `invalid value encountered in divide`**
- This can appear when evaluating the velocity field exactly at a rotor center (r = 0). The
  warning is harmless; the code clamps the singularity.

### 5.10 Project structure

```
cfd/
├── morph_cfd.py              # Main script: config geometry, flow model, and all five figures
├── configs.py                # EDF positions and thrust values for X, O, T, H
├── flow_model.py             # Actuator-disk / vortex-tube velocity field computation
├── plotting.py               # Shared Matplotlib formatting and color definitions
├── requirements.txt          # pip dependencies (numpy, matplotlib, scipy)
└── results/                  # Default output directory for generated figures
    ├── fig1_topdown_velocity.png
    ├── fig2_sideview_velocity.png
    ├── fig3_perturbation.png
    ├── fig4_interrotor_profiles.png
    └── fig5_summary.png
```

---

## Team documents

Folder: `Documentation/`

| File | Contents |
|------|----------|
| `Dependencies & How to Run.docx` | Original setup notes for the Simscape model. It names `Quadcopter_Drone.prj`; the project file is now `transfomer_drone.prj`. |
| `Tasking for Simulation Team 3-7-2026.docx` | Simulation team work items |
| `Firday_3_6_2026_Presentation.pptx` | Simulation status presentation (March 6, 2026) |

---

## Repository housekeeping

Generated files and caches shouldn't be committed. Suggested `.gitignore` at the repo root:

```gitignore
# Part 1: ROS 2 workspace build output
morphing_drone_ws/build/
morphing_drone_ws/install/
morphing_drone_ws/log/
__pycache__/
*.pyc

# Part 2: MATLAB / Simulink
# work/ is the project's cache folder; keep the empty folder but not its contents
Simscape_Simulation/work/*
!Simscape_Simulation/work/.gitkeep
*.asv
*.autosave
*.slxc
slprj/

# Part 3.2: ardupilot (morph simulator)
ardupilot/node_modules/
ardupilot/dist/

# Part 3.1: drone-sim (heavy-lift estimator)
drone-sim/node_modules/
drone-sim/dist/
```

To stop tracking files that are already committed (this keeps them on your disk):

```bash
touch Simscape_Simulation/work/.gitkeep
git rm -r --cached --ignore-unmatch morphing_drone_ws/build morphing_drone_ws/install \
  morphing_drone_ws/log Simscape_Simulation/work drone-sim/node_modules ardupilot/node_modules "*.asv"
git add .gitignore Simscape_Simulation/work/.gitkeep
git commit -m "Stop tracking build output and caches"
```

`--ignore-unmatch` skips any path that isn't tracked, so the command is safe to run as-is.

---

## Credits

- Cal Poly Pomona ECE Senior Design.
- **Simscape model:** Kaung Nyi Naing, Keeyan Canlas, and Troy Tadros (simulation team), based on
  [Quadcopter Drone Model in Simscape](https://www.mathworks.com/matlabcentral/fileexchange/63580-quadcopter-drone-model-in-simscape)
  © The MathWorks, Inc.
