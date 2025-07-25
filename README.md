# 🤖 Autonomous Navigation Robot

An indoor autonomous robot built using:
- 🧠 **Raspberry Pi 5** running **ROS 2 (Jazzy)** with Python nodes
- ⚙️ **Arduino Nano** for real-time motor control
- 🔍 **LiDAR (RPLiDAR A1)** for SLAM and obstacle detection

---

<pre lang="markdown"> ## 🗂️ Repository Structure ``` navigation-robot/ ├── robot_ws/ # ROS 2 workspace │ ├── src/ # ROS 2 packages (Python nodes) │ ├── scripts/ # Shell scripts to run and setup │ ├── build/ # Auto-generated build files (ignored) │ ├── install/ # Auto-generated install files (ignored) │ └── log/ # ROS logs (ignored) ├── arduino/ # Arduino sketch for motor control ├── images/ # Robot images and wiring diagrams ├── .gitignore ├── LICENSE └── README.md ``` </pre>

## ✨ Features

- Real-time SLAM using RPLiDAR A1
- ROS 2 (Jazzy) based Python nodes for autonomous navigation
- Serial communication with Arduino for motor control
- Modular and extensible ROS 2 package structure
- Shell scripts for quick setup and launch

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

