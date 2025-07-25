# 🤖 Autonomous Navigation Robot

An indoor autonomous robot built using:
- 🧠 **Raspberry Pi 5** running **ROS 2 (Jazzy)** with Python nodes
- ⚙️ **Arduino Nano** for real-time motor control
- 🔍 **LiDAR (RPLiDAR A1)** for SLAM and obstacle detection

---
<pre> ## 🗂️ Repository Structure ``` navigation-robot/ ├── robot_ws/ # ROS 2 workspace │ ├── src/ # ROS 2 packages (Python nodes) │ ├── scripts/ # Shell scripts to run and setup │ ├── build/ # Auto-generated build files (ignored) │ ├── install/ # Auto-generated install files (ignored) │ └── log/ # ROS logs (ignored) ├── arduino/ # Arduino sketch for motor control ├── images/ # Robot images and wiring diagrams ├── .gitignore ├── LICENSE └── README.md ``` </pre>

## ✨ Features

- 🔍 **SLAM Mapping** using [Cartographer](https://google-cartographer.readthedocs.io/) for real-time map generation
- 📍 **Localization** with Adaptive Monte Carlo Localization (AMCL)
- 🧭 **Path Planning** using:
  - Global Planner: **Dijkstra**
  - Local Planner: **Dynamic Window Approach (DWA)**
- 🧠 Built on **ROS 2 Jazzy** (Python nodes)
- ⚙️ **Serial communication** with Arduino Nano for real-time motor control
- 🛠️ Fully automated launch using custom shell scripts
- 💡 Modular ROS 2 workspace (`robot_ws/`) and Arduino code (`arduino/`)

---
## 🚀 Getting Started

### 🔧 Prerequisites

- Raspberry Pi 5 with **Ubuntu 22.04**
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- Python 3.10+
- `colcon` and `rosdep` installed
- RPLiDAR A1 connected via USB
- Arduino Nano 

---

## ⚙️ Setup Instructions

### 🧠 Raspberry Pi / ROS 2 Setup

1. Clone the repo:

## 🔌 Arduino Setup

The Arduino Nano controls the robot's motors based on commands received from the Raspberry Pi via a serial (USB) connection.

### ✅ Requirements

- **Arduino Nano**
- **Motor driver** (e.g. L298N or similar)
- Installed [Arduino CLI](https://arduino.github.io/arduino-cli/latest/installation/) or [Arduino IDE](https://www.arduino.cc/en/software)

### 🔧 Uploading the Code

#### 📦 Option 1: Using Arduino CLI

1. Open a terminal in the root of your project
2. Compile the code:
   ```bash
   arduino-cli compile --fqbn arduino:avr:nano arduino/navigation_code.ino

