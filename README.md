LoCoBot Control Node: WASD Control for Arm, Gripper, and Base
Overview

This ROS 2 node allows you to control the LoCoBot robot’s arm, gripper, and base using the WASD keys for movement and additional keys for gripper control. The node subscribes to the necessary topics for controlling the arm, gripper, and base and sends appropriate commands based on keyboard inputs.

WASD keys are used to move the robot's base.

O and C keys control the gripper (open and close).

The robot’s arm will send a basic movement command to move the arm to a predefined position, which can be expanded later for specific control.

Features

Base movement:

W: Move forward

S: Move backward

A: Turn left

D: Turn right

Gripper control:

O: Open gripper

C: Close gripper

Arm control: Sends a basic trajectory for the arm to a predefined pose (can be extended later for dynamic control).

Prerequisites

ROS 2 (Foxy, Galactic, or later)

Ensure that ROS 2 is installed and properly set up on your system.

Follow the official ROS 2 installation guide
 if needed.

LoCoBot Bringup:

You must have the LoCoBot bringup running so that the arm, gripper, and base controllers are active.

Launch the LoCoBot bringup using the following command:

ros2 launch interbotix_xs_modules locobot_bringup.launch.py


pygame:

This node requires pygame to capture keyboard input.

Install it with the following command:

pip install pygame

Setup

Clone the repository (if you haven't already) into your workspace:

cd ~/locobot_ws/src
git clone <repository-url> locobot_control


Build the package:

From the root of your ROS 2 workspace:

cd ~/locobot_ws
colcon build --symlink-install


Source the workspace:

source install/setup.bash

Running the Node

Ensure that the LoCoBot bringup is running before starting the control node.

The following topics should be active:

/locobot/arm_controller/commands

/cmd_vel

/locobot/gripper/set_gripper_position

Run the node:

Start the control node with the following command:

ros2 run locobot_control locobot_control_node


Control the robot:

Use the WASD keys to control the base:

W: Move forward

S: Move backward

A: Turn left

D: Turn right

Use O and C to control the gripper:

O: Open the gripper

C: Close the gripper

Stop the node:
Press Ctrl+C to stop the node at any time.

Extending the Node

This node can be easily extended to include more advanced features, such as:

Arm control: Implementing more sophisticated control for the arm, such as moving to specific joint positions or using inverse kinematics.

Base velocity: Adding user-configurable velocity parameters for fine-tuned control.

Multiple gripper modes: Allowing finer control of the gripper, such as controlling its position based on force feedback or other inputs.

To implement these changes, simply modify the code within the locobot_control_node.py file.

Troubleshooting

No response from base or gripper:

Ensure that the LoCoBot bringup is active.

Check if the topics /cmd_vel and /locobot/gripper/set_gripper_position are being published.

Ensure pygame is installed and can capture keypresses.

Gripper service not available:

If the gripper service is not available, verify that the gripper node is running and that the service /locobot/gripper/set_gripper_position is accessible.

Keyboard not responding:

Make sure that your terminal window has focus so that keypresses are captured.

License

This code is licensed under the MIT License. See the LICENSE file for more information.

Acknowledgments

LoCoBot: A mobile manipulation platform developed by Interbotix
.

ROS 2: The open-source robotics framework.

pygame: A cross-platform set of Python modules designed for writing video games, used here for keyboard input.

Feel free to modify or extend this node according to your project needs. If you encounter issues or want to contribute enhancements, feel free to open a pull request or submit an issue!
