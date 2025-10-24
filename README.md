# 🦾 LoCoBot Control Node: WASD Control for Arm, Gripper, and Base

## Overview

This ROS 2 node enables manual control of the **LoCoBot’s base, arm, and gripper** using keyboard input.  
You can use the **WASD keys** for base movement and additional keys for **gripper** control.  
The node publishes commands to the appropriate ROS 2 topics to drive the robot.

---

## 🎮 Features

### Base Movement
| Key | Action        |
|-----|----------------|
| **W** | Move forward  |
| **S** | Move backward |
| **A** | Turn left     |
| **D** | Turn right    |

### Gripper Control
| Key | Action         |
|-----|----------------|
| **O** | Open gripper  |
| **C** | Close gripper |

### Arm Control
- Sends a **predefined trajectory** command to move the arm to a basic pose.  
- Can be extended later for **dynamic arm control**.

---

## ⚙️ Prerequisites

### 1. ROS 2 (Foxy, Galactic, Humble, or later)
Ensure that ROS 2 is installed and properly configured.  
Follow the official [ROS 2 installation guide](https://docs.ros.org/en/foxy/Installation.html) if needed.

### 2. LoCoBot Bringup
The LoCoBot bringup must be running so that the arm, gripper, and base controllers are active.

```bash
ros2 launch interbotix_xs_modules locobot_bringup.launch.py
