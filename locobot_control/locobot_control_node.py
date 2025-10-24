#!/usr/bin/env python3
"""
locobot_control_node.py
-----------------------
Control the LoCoBot using WASD for base movement and keypresses for gripper control.
"""

import rclpy
import pygame
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

class LoCoBotControlNode(Node):
    def __init__(self):
        super().__init__('locobot_control_node')

        # Initialize pygame
        pygame.init()

        # Create the screen (it's not actually displayed, just for keyboard capture)
        self.screen = pygame.display.set_mode((100, 100))
        pygame.display.set_caption("LoCoBot Control")

        # Publisher for base movement (cmd_vel)
        self.base_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for arm command topic
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/locobot/arm_controller/commands',
            10
        )

        # Service client to control the gripper (open/close)
        self.gripper_client = self.create_client(Trigger, '/locobot/gripper/set_gripper_position')

        # Arm joint names (adjust these as needed for your setup)
        self.joint_names = [
            'waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate'
        ]

        # Initialize movement variables
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Timer to keep base and gripper control going
        self.timer = self.create_timer(0.1, self.update_controls)

        self.get_logger().info("LoCoBot control node started.")

    def update_controls(self):
        # Handle the keyboard input
        self.handle_keyboard_input()

        # Send base velocity command
        self.move_base()

        # Control the gripper (open/close)
        self.control_gripper()

    def handle_keyboard_input(self):
        # Handle events from pygame to check key presses
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()

        # Define control mapping (WASD for base movement)
        keys = pygame.key.get_pressed()
        
        # WASD for controlling the base movement
        if keys[pygame.K_w]:  # Forward
            self.linear_velocity = 0.1
            self.angular_velocity = 0.0
        elif keys[pygame.K_s]:  # Backward
            self.linear_velocity = -0.1
            self.angular_velocity = 0.0
        elif keys[pygame.K_a]:  # Turn left
            self.linear_velocity = 0.0
            self.angular_velocity = 0.2
        elif keys[pygame.K_d]:  # Turn right
            self.linear_velocity = 0.0
            self.angular_velocity = -0.2
        else:
            # Stop the robot if no keys are pressed
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

        # Gripper control (O for open, C for close)
        if keys[pygame.K_o]:  # Open gripper
            self.control_gripper(True)
        elif keys[pygame.K_c]:  # Close gripper
            self.control_gripper(False)

        # Optionally add arm control keys here (e.g., 1, 2, 3 to move arm to specific positions)
        # For example: if keys[pygame.K_1]: move_arm_to_pose(1)

    def move_base(self):
        # Create a Twist message for base movement
        move_cmd = Twist()
        move_cmd.linear.x = self.linear_velocity  # Forward/backward movement
        move_cmd.angular.z = self.angular_velocity  # Left/right turning

        # Publish the base movement command
        self.base_pub.publish(move_cmd)
        self.get_logger().info(f"Base movement: linear.x={self.linear_velocity}, angular.z={self.angular_velocity}")

    def control_gripper(self, open_gripper):
        # Wait for gripper service to be available
        if not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Gripper service not available')
            return

        # Create a service request
        req = Trigger.Request()

        # If open_gripper is True, open the gripper; otherwise, close it
        if open_gripper:
            self.get_logger().info('Sending gripper open command.')
        else:
            self.get_logger().info('Sending gripper close command.')

        # Call the service to control the gripper
        future = self.gripper_client.call_async(req)
        future.add_done_callback(self.gripper_response_callback)

    def gripper_response_callback(self, future):
        # Callback to handle the gripper service response
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Gripper command successful: {response.message}')
            else:
                self.get_logger().warn(f'Gripper command failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Failed to call gripper service: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LoCoBotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3