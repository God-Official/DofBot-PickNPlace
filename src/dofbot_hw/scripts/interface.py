#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from Arm_Lib import Arm_Device  # Make sure your ArmLib is installed
import math


class MoveItToArm(Node):
    def __init__(self):
        super().__init__('moveit_to_arm')

        # Subscribe to controller states
        self.arm_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/state',
            self.state_callback,
            10
        )

        self.gripper_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/gripper_controller/state',
            self.stateg_callback,
            10
        )
        self.g_angle = 0

        # Initialize the ArmLib device
        self.arm = Arm_Device()
        self.get_logger().info("ArmLib Device Initialized ✅")

        # Keep track of last commanded positions to avoid spamming
        self.last_positions = []
        self.lastg_positions = []

    def stateg_callback(self, msg: JointTrajectoryControllerState):
        desired_positions = msg.desired.positions

        if not desired_positions:
            return
        if desired_positions == self.lastg_positions:
            return

        # Convert radians → degrees for the gripper
        angle_deg = -(int(math.degrees(desired_positions[0]))) + 75
        self.g_angle = angle_deg


        self.get_logger().info(f"Sending gripper angle: {angle_deg}")

        # Send to servo 6 (adjust if your gripper uses another channel)
        self.arm.Arm_serial_servo_write(6, angle_deg, 500)

        # Update last gripper position
        self.lastg_positions = desired_positions

    def state_callback(self, msg: JointTrajectoryControllerState):
        desired_positions = msg.desired.positions

        if not desired_positions:
            return
        if desired_positions == self.last_positions:
            return

        # Convert radians → degrees
        angles_deg = [int(math.degrees(p)) for p in desired_positions]
        self.get_logger().info(f"Sending angles to Arm: {angles_deg}")

        # Execute the angles using ArmLib
        self.arm.Arm_serial_servo_write6(
            int(angles_deg[0] + 90),
            int(angles_deg[1] + 90),
            int(angles_deg[2] + 90),
            int(angles_deg[3] + 90),
            int(angles_deg[4] + 90),
            int(self.g_angle),  # optional 6th joint if your arm has only 5
            500
        )

        self.last_positions = desired_positions


def main():
    rclpy.init()
    node = MoveItToArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arm.Arm_serial_disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()