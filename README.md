# 🐢 TurtleBot2 Trajectory Tracking with ROS 2 (PID & Lyapunov Control)

This repository contains ROS 2 packages and Python scripts to simulate and control a **TurtleBot2** in a square trajectory using two control strategies: **PID** and **Lyapunov-based control**.

---

## 🧠 Overview

We implement and compare two control strategies to follow a predefined square trajectory:

- ✅ **PID Controller**: Classical proportional–integral–derivative control
- ✅ **Lyapunov-based Controller**: Based on Lyapunov stability theory for nonlinear systems

Simulation is carried out on **Ubuntu 22.04** with **ROS 2 Humble**.

---

## 🧰 Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- `turtlebot2` packages (navigation, robot_state_publisher, etc.)
- `rqt_graph`, `rviz2`, `turtlesim` (for visualization/debugging)

Install dependencies:
```bash
sudo apt update
sudo apt install ros-humble-turtlebot2* ros-humble-rqt* ros-humble-rviz2
```
