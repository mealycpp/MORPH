# Morphing EDF Quadcopter Simulation

ROS 2 + Gazebo + Reinforcement Learning on WSL2 (Windows 11)

## Project Overview

A **shape-morphing EDF quadcopter** that transforms between four configurations
mid-flight using servo-actuated arm joints. The simulation collects flight data
for each shape, compares performance, and includes an RL system that learns
optimal transition timing.

### Configurations

| Config | Servo Angles [FR, FL, RL, RR] | Description |
|--------|-------------------------------|-------------|
| **X**  | [0, 0, 0, 0]                 | Standard quadcopter — all arms straight |
| **O**  | [+90, +90, +90, +90]         | Compact circular — CCW pinwheel |
| **T**  | [-45, +45, 0, 0]             | T-shape — front pair spreads, rear stays |
| **H**  | [+45, -45, +45, -45]         | H-shape — parallel arm pairs |

### Hardware Design

- **EDF Motor:** D2842-2800KV 70mm ducted fan (4S LiPo)
- **Max thrust per EDF:** ~1.82 kg (17.86 N)
- **Servo:** 35KG coreless (x4), 300°/s, ±90° range
- **Frame:** 150×150 mm carbon fiber center plate
- **Arms:** 318 mm total (159 mm inner + 159 mm outer, split at servo)
- **Estimated AUW:** ~2.5 kg
- **Thrust-to-weight ratio:** ~2.9:1

---

## 1. Prerequisites — WSL2 Setup

### 1.1 Install WSL2 + Ubuntu 22.04
```powershell
# In Windows PowerShell (Admin)
wsl --install -d Ubuntu-22.04
```

### 1.2 Enable GUI
WSLg is built into Windows 11 — Gazebo GUI works out of the box.
If you have GPU issues, install the WSL GPU driver from NVIDIA/AMD/Intel.

### 1.3 Install ROS 2 Humble
```bash
# Inside WSL Ubuntu 22.04
sudo apt update && sudo apt upgrade -y

# Add ROS 2 repo
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu jammy main" \
  > /etc/apt/sources.list.d/ros2.list'

sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-controller-manager ros-humble-joint-trajectory-controller \
  ros-humble-effort-controllers python3-colcon-common-extensions \
  python3-rosdep python3-pip

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Initialize rosdep
sudo rosdep init 2>/dev/null; rosdep update
```

### 1.4 Install Python dependencies
```bash
pip3 install numpy pandas matplotlib scipy --break-system-packages
```

---

## 2. Build the Workspace

```bash
cd ~/morphing_drone_ws

# Clean any previous build artifacts
rm -rf build/ install/ log/

# Source ROS 2
source /opt/ros/humble/setup.bash

# Install missing ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build (this is a pure ament_python package — no cmake)
colcon build --symlink-install

# Source the workspace
source install/setup.bash
echo "source ~/morphing_drone_ws/install/setup.bash" >> ~/.bashrc
```

### Verify the build
```bash
ros2 pkg executables morphing_drone
```
Should show: `flight_controller`, `morph_controller`, `data_logger`,
`analyze_flight_data`, `run_morph_sequence`, `rl_controller`

---

## 3. Launch the Simulation

### 3.1 Full simulation (Gazebo + controllers + morph sequence)
```bash
ros2 launch morphing_drone full_simulation.launch.py
```

### 3.2 What happens
1. Gazebo loads the world with the drone at 0.5 m
2. Flight controller takes off and stabilizes at 5 m in **X-config**
3. After 10 s settling, the morph sequence begins:
   - **X-config** hover for 30 s (data collection)
   - Morph to **O-config** → hover 30 s
   - Morph to **T-config** → hover 30 s
   - Morph to **H-config** → hover 30 s
4. Flight data saved to `~/morphing_drone_data/flight_data.csv`

### 3.3 Analyze the data (after simulation completes)
```bash
ros2 run morphing_drone analyze_flight_data
```
Generates `~/morphing_drone_data/flight_analysis.pdf` with:
- Altitude hold performance per config
- Roll/pitch stability comparison
- Power consumption
- Motor thrust distributions
- Summary bar charts

---

## 4. Machine Learning — RL-Based Morph Optimization

The RL system trains a neural network to keep the drone stable during shape
transitions and discovers optimal morph speeds. It uses its own lightweight
physics sim (no Gazebo needed) that runs 1000x faster than real-time.

### 4.1 Install ML dependencies
```bash
pip install gymnasium stable-baselines3[extra] torch --break-system-packages
```

### 4.2 Train the agent
```bash
cd ~/morphing_drone_ws/src/morphing_drone/morphing_drone/ml

# Quick test (~5 min, lower quality but verifies everything works)
python3 train_agent.py --quick

# Full training (~30 min, production quality)
python3 train_agent.py

# Resume interrupted training
python3 train_agent.py --resume

# Evaluate an existing model without retraining
python3 train_agent.py --eval
```

### 4.3 What the agent learns
- How to adjust motor thrusts to stay level during morphing
- Optimal morph speed for each transition pair (X→O, X→T, etc.)
- Anticipatory corrections before geometry changes
- How to handle CG shifts and inertia changes smoothly

### 4.4 Training output
```
~/morphing_drone_data/ml/
  best_model.zip            # Trained neural network weights
  final_model.zip           # Final checkpoint
  eval_results.json         # Per-transition performance metrics
  morph_timing_table.json   # Learned optimal transition times
  training_log.csv          # Reward curves
  tb_logs/                  # TensorBoard logs
  checkpoints/              # Periodic checkpoints
```

### 4.5 Use the trained model in Gazebo
After training, deploy the learned policy as a ROS 2 node:
```bash
# Hybrid mode: PID handles hover, RL takes over during morph
ros2 run morphing_drone rl_controller --ros-args -p mode:=hybrid
```

### 4.6 Monitor training with TensorBoard (optional)
```bash
pip install tensorboard --break-system-packages
tensorboard --logdir ~/morphing_drone_data/ml/tb_logs
# Open http://localhost:6006 in your browser
```

---

## 5. ArduPilot Tuning Simulator

A separate React-based simulator (`morphing_drone_ardupilot_sim.jsx`) generates
ArduPilot tuning parameters for each configuration. It includes:

- **Universal parameters** — safe across all shapes (conservative gains
  tuned for worst-case control authority)
- **Per-config parameters** — optimal gains for each individual shape
- **Morph transition analysis** — safe transition times, altitude budgets,
  CG shift calculations, and risk ratings for every config pair
- **Flight data charts** — thrust, power, current, RPM, efficiency vs throttle

Open the `.jsx` file as a Claude artifact or in any React environment.

---

## 6. Project Structure

```
morphing_drone_ws/
├── README.md
└── src/morphing_drone/
    ├── package.xml              # ament_python package manifest
    ├── setup.py                 # Python package config with data_files
    ├── setup.cfg                # ament_python install paths
    ├── resource/
    │   └── morphing_drone       # ament index marker
    ├── config/
    │   ├── drone_params.yaml    # Physical parameters from design doc
    │   └── pid_gains.yaml       # PID controller tuning
    ├── launch/
    │   └── full_simulation.launch.py
    ├── urdf/
    │   └── morphing_drone.urdf.xacro  # Parametric URDF with servo joints
    ├── worlds/
    │   └── empty_flight.world
    └── morphing_drone/               # Python package
        ├── __init__.py
        ├── flight_controller.py      # PID attitude + altitude control
        ├── morph_controller.py       # Servo joint commands (X/O/T/H)
        ├── data_logger.py            # Records flight data to CSV
        ├── analyze_flight_data.py    # Post-processing & comparison plots
        ├── run_morph_sequence.py     # Orchestrates X → O → T → H test
        └── ml/                       # Machine learning subsystem
            ├── __init__.py
            ├── morph_drone_env.py    # Gymnasium env (standalone physics)
            ├── train_agent.py        # PPO training script
            └── rl_controller.py      # ROS 2 node for deploying trained model
```

---

## 7. Configuration Details

### Morphing Mechanism
Each arm has a **revolute joint** at the servo mount (159 mm from center).
The outer segment (159 mm, carrying the EDF) rotates ±90° to morph the shape.
All four servos can move simultaneously or independently.

### T-Config Design Choice: [-45, +45, 0, 0]
The T-config only moves the front two servos (FR sweeps right, FL sweeps left)
while the rear two stay in their X-config diagonal positions. This was chosen
over the alternatives after analysis showed:
- **Lowest CG offset:** 56 mm (vs 80 mm for [-45,+45,+45,-45])
- **Best pitch authority:** 110% of X-config baseline
- **Simpler transition:** Only 2 servos move instead of 4
- **Higher trim headroom:** 2.2:1 corrective torque ratio

### Coordinate Frame
- X: forward, Y: left, Z: up
- Yaw: rotation about Z
- Servo angles: positive = counterclockwise when viewed from above

---

## 8. Data Collected Per Configuration

| Metric | Unit | Description |
|--------|------|-------------|
| Position (x, y, z) | meters | World frame |
| Orientation (roll, pitch, yaw) | radians | Euler angles |
| Linear velocity | m/s | Body frame |
| Angular velocity | rad/s | Body frame |
| Motor commands | N | Per-EDF thrust |
| Power consumption | W | Estimated from thrust |
| Stability index | — | RMS of roll+pitch deviation |

---

## 9. Troubleshooting

**colcon build fails?**
- Make sure you sourced ROS 2: `source /opt/ros/humble/setup.bash`
- Clean previous build: `rm -rf build/ install/ log/`
- Install deps: `rosdep install --from-paths src --ignore-src -r -y`
- This is an `ament_python` package (no cmake) — don't create CMakeLists.txt

**Gazebo won't open in WSL?**
- Ensure WSLg is working: `xclock` should show a clock window
- If not, update Windows 11 and WSL: `wsl --update`

**Drone flips on takeoff?**
- Check `pid_gains.yaml` — reduce P gains
- Verify EDF thrust directions in URDF (should be +Z in body frame)

**ML training fails?**
- Install deps: `pip install gymnasium stable-baselines3[extra] torch --break-system-packages`
- Check Python version: needs Python 3.8+
- If CUDA errors, training will fall back to CPU (slower but works)

**rl_controller says "No model found"?**
- Run training first: `python3 train_agent.py --quick`
- Model saves to `~/morphing_drone_data/ml/best_model.zip`
