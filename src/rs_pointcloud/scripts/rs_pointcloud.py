#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud


class DepthRGBToPointCloud(Node):
    def __init__(self):
        super().__init__("depth_rgb_to_pointcloud")

        self.get_logger().info("✅ Organized Pointcloud Node Started")

        self.bridge = CvBridge()

        self.color_frame = None
        self.depth_frame = None
        self.cam_info = None

        # COLOR IMAGE
        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.color_callback,
            10
        )

        # DEPTH IMAGE
        self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_callback,
            10
        )

        # CAMERA INFO
        self.create_subscription(
            CameraInfo,
            "/camera/camera/aligned_depth_to_color/camera_info",
            self.info_callback,
            10
        )

        # OUTPUT POINTCLOUD
        self.pc_pub = self.create_publisher(
            PointCloud2,
            "/camera/custom_pointcloud",
            10
        )

    def info_callback(self, msg):
        self.cam_info = msg

    def color_callback(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        if self.cam_info is None:
            return

        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        if self.color_frame is None:
            return

        self.generate_pointcloud()

    def generate_pointcloud(self):
        depth = self.depth_frame.astype(np.float32) / 1000.0
        color = self.color_frame

        h, w = depth.shape

        fx, fy = self.cam_info.k[0], self.cam_info.k[4]
        cx, cy = self.cam_info.k[2], self.cam_info.k[5]

        u, v = np.meshgrid(np.arange(w), np.arange(h), indexing='xy')

        Z = depth.reshape(-1)
        X = ((u - cx) * depth).reshape(-1) / fx
        Y = ((v - cy) * depth).reshape(-1) / fy

        # RGB pack
        rgb = color.reshape(-1, 3)
        rgb_uint32 = (
            (rgb[:, 2].astype(np.uint32) << 16) |
            (rgb[:, 1].astype(np.uint32) << 8) |
            (rgb[:, 0].astype(np.uint32))
        )
        rgb_float = rgb_uint32.view(np.float32)

        pts = np.column_stack((X, Y, Z, rgb_float))

        fields = [
            PointField(name="x",   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        header = self.cam_info.header
        header.frame_id = "camera_link_optical"
        header.stamp.sec = 0
        header.stamp.nanosec = 0

        cloud = create_cloud(header, fields, pts)
        cloud.height = h
        cloud.width = w
        cloud.is_dense = False

        self.pc_pub.publish(cloud)
        self.get_logger().info(f"Published organized pointcloud: {w} x {h}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthRGBToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
