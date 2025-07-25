# 🤖 Autonomous Navigation Robot

An indoor autonomous robot built using:
- 🧠 **Raspberry Pi 5** running **ROS 2 (Jazzy)** with Python nodes
- ⚙️ **Arduino Nano** for real-time motor control
- 🔍 **LiDAR (RPLiDAR A1)** for SLAM and obstacle detection

---

## 🗂️ Repository Structure

<pre>
navigation-robot/
├── robot_ws/        # ROS 2 workspace
│   ├── src/         # ROS 2 packages (Python nodes)
│   ├── scripts/     # Shell scripts to run and setup
│   ├── build/       # Auto-generated build files (ignored)
│   ├── install/     # Auto-generated install files (ignored)
│   └── log/         # ROS logs (ignored)
├── arduino/         # Arduino sketch for motor control
├── images/          # Robot images and wiring diagrams
├── .gitignore
├── LICENSE
└── README.md
</pre>

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

## 🔌 Arduino Setup

The Arduino Nano controls the robot's motors based on commands received from the Raspberry Pi via a serial (USB) connection.

### ✅ Requirements

- **Arduino Nano**
- **Motor driver** (e.g. L298N or similar)
- Installed [Arduino CLI](https://arduino.github.io/arduino-cli/latest/installation/) or [Arduino IDE](https://www.arduino.cc/en/software)

### 🔧 Uploading the Code

#### Using Arduino CLI

1. Open a terminal in the root of your project
2. Compile the code:
   ```bash
   arduino-cli compile --fqbn arduino:avr:nano arduino/navigation_code.ino
3. Upload the code:
```bash
  arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano  ```



## 🧠 Raspberry Pi / ROS 2 Setup

After connecting the hardware:

- **Arduino Nano** to the Raspberry Pi via USB  
- **RPLiDAR A1** via USB  

You can run the following shell scripts from `robot_ws/scripts/` to operate the robot:

1. **Start autonomous movement and obstacle avoidance**  
    ```bash
    ./robot_move.sh
    ```
    This will launch the robot movement node, visualize real-time LiDAR scans in **RViz**, and enable obstacle avoidance.

2. **Start SLAM and build a map**  
    ```bash
    ./show_map.sh
    ```
    This launches Cartographer SLAM and shows the live map in **RViz**.

3. **Save the generated map**  
    ```bash
    ./save_map.sh
    ```

4. **Stop robot motion**  
    After mapping, stop the robot’s movement to prevent interference.  
    (The LiDAR process should remain running.)

5. **View robot model (URDF)**  
    ```bash
    ./model.sh
    ```

6. **Start localization**  
    ```bash
    ./localization.sh
    ```

7. **Enable global path planning**  
    ```bash
    ./globalization.sh
    ```

8. **Send navigation goals (point-to-point)**  
    ```bash
    ./point_to_point.sh
    ```
    You can send goal points via **RViz’s "2D Nav Goal"** or the terminal.

9. **Optional: Run coverage mode**  
    For autonomous full-room coverage, launch:  
    ```bash
    ros2 run coverage coverage_node
    ```

> 📝 **Note:** Make sure you have sourced the ROS 2 workspace and installed all dependencies before running these commands.

