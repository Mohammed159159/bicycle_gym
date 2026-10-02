# ARL Autonomous Vehicle Control Track

**Autotronics Research Lab (ARL) — Ain Shams University**  
*Course: Autonomous Vehicles & Drive-by-Wire Systems | Individual Project*

---

## 📖 Hands-On Assignment Guide

All instructions, mathematical derivations, milestone walkthroughs, and deliverable specifications are detailed in the official student task guide:

👉 **[Read the Full Hands-On Assignment Guide (TASK.md)](./TASK.md)** 👈

---

## 🏎️ Vehicle Physics Model

This project implements an **Extended Kinematic Bicycle Model** (4-DOF: $x, y, \theta, v$) with Drive-by-Wire powertrain dynamics:
- **State Vector** $\mathbf{x} = [x, y, \theta, v]^T \in \mathbb{R}^4$: Cartesian coordinates of the rear axle center $(x, y)$, heading angle $\theta$, and forward velocity $v$.
- **Control Input Vector** $\mathbf{u} = [u_{throttle}, \delta]^T \in \mathbb{R}^2$: Normalized powertrain throttle/braking effort $u_{throttle} \in [-1.0, 1.0]$ (scaled by acceleration gain $k_a$) and front wheel steering angle $\delta \in [-\delta_{max}, \delta_{max}]$.
- **Physical Resistance**: Accounts for motor drive torque, mechanical braking, rolling friction ($c_{roll} v$), and speed-squared aerodynamic drag ($c_{drag} v^2$).

---

## 🚀 Quickstart

### 1. Build the Workspace
```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Install dependencies (SciPy is required for MPC optimization)
sudo apt update && sudo apt install -y python3-scipy python3-numpy

# Build the workspace (bicycle_sim, bicycle_control, track_environment)
cd ~/Projects/content/ARL\ Sessions\ -\ Control/contorl_task
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Modes

| Mode | Launch Command | Section |
|---|---|---|
| **Base Simulation (CLI Testing)** | `ros2 launch bicycle_sim bicycle_sim.launch.py` | Milestones 1 & 2 |
| **Interactive Keyboard Teleop** | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop` | Milestones 3 & 4 |
| **Lateral PID (Reactive)** | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=lateral_pid` | Milestone 5.2 |
| **Pure Pursuit (Geometric Preview)**| `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=pure_pursuit` | Milestone 5.3 |
| **Extended Kinematic MPC (Optimal Preview)** | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=mpc` | Milestone 5.4 |

### 3. Real-Time Telemetry & Graphing
```bash
# Install plotting and telemetry visualization tools
sudo apt update && sudo apt install -y ros-humble-plotjuggler-ros rqt-plot

# Inspect live signals in rqt_plot:
ros2 run rqt_plot rqt_plot /telemetry/cte /telemetry/speed

# Or launch PlotJuggler for multi-topic time-series analysis:
ros2 run plotjuggler plotjuggler
```

---

## 🎯 Milestones at a Glance

- **Milestone 1**: Topic Discovery, Graph Inspection & Telemetry Plotting (`ros2 topic list / info`, `rqt_plot`, `plotjuggler`)
- **Milestone 2**: Extended Kinematic Bicycle Model & Euler Integration (`src/bicycle_sim/bicycle_sim/bicycle_model.py`)
- **Milestone 3**: Teleoperation Bridge & Open-Loop Driving (`src/bicycle_control/bicycle_control/teleop_bridge.py`)
- **Milestone 4**: Low-Level Powertrain Cruise Control (`src/bicycle_control/bicycle_control/longitudinal_pid.py`)
- **Milestone 5**: Autonomous Path Tracking — It's Time to Get the Car to Drive Autonomously!
  - **5.1**: High-Level Velocity Profiler & Path Curvature (`src/bicycle_control/bicycle_control/velocity_profiler.py`)
  - **5.2**: Steer Using Reactive Feedback (`src/bicycle_control/bicycle_control/lateral_pid.py`)
  - **5.3**: Steer Using Geometric Preview (`src/bicycle_control/bicycle_control/pure_pursuit.py`)
  - **5.4**: Steer Using Constrained Optimal Preview (Extended Kinematic MPC) (`src/bicycle_control/bicycle_control/mpc.py`)
  - **5.5**: Real-Time Telemetry, Graphing & RViz Dashboard Engineering (`src/track_environment/track_environment/lap_analyzer.py`)
- **Milestone 6**: Free Exploration & Reference Resources (Ackermann Kinematics, 3D Simulation, Nav2 MPPI)
- **Milestone 7**: Deliverable 1 — Repository Documentation (`README.md` Benchmark Report)
- **Milestone 8**: Deliverable 2 — Technical Video Walkthrough (3–5 Minute Demo)

---

## 🏆 Telemetry Benchmark Leaderboard

*(To be completed by the student as part of Milestone 7)*

| Controller Mode | Best Lap Time (s) | Top Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Laps Completed / Status |
|---|---|---|---|---|---|---|
| **Manual Teleoperation** | — | — | — | — | — | — |
| **Lateral PID (Reactive)** | — | — | — | — | — | — |
| **Pure Pursuit (Preview)** | — | — | — | — | — | — |
| **Extended Kinematic MPC (Optimal)** | — | — | — | — | — | — |
