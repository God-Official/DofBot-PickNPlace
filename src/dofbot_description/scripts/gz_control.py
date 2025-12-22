#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MoveitToGazeboBridge(Node):
    def __init__(self):
        super().__init__("moveit_to_gz_bridge")

        # MoveIt fake controller joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10
        )

        # Gazebo real controller command
        self.pub_gz = self.create_publisher(
            JointTrajectory,
            '/arm_gz_controller/joint_trajectory',
            10
        )

        self.get_logger().info("🔄 MoveIt → Gazebo bridge running!")

    def callback(self, msg: JointState):
        jt = JointTrajectory()
        jt.joint_names = msg.name

        point = JointTrajectoryPoint()
        point.positions = msg.position
        point.time_from_start.sec = 1

        jt.points.append(point)

        self.pub_gz.publish(jt)

        self.get_logger().info(f"➡️ Sent to Gazebo: {list(msg.position)}")


def main(args=None):
    rclpy.init(args=args)
    node = MoveitToGazeboBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
