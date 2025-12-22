#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ColorTFPointCloud(Node):
    def __init__(self):
        super().__init__('color_tf_from_pointcloud')

        self.bridge = CvBridge()
        self.rgb_img = None
        self.pc = None

        # --- SUBSCRIBERS ---
        self.create_subscription(Image,
                                 "/rgbd_camera/image",
                                 self.rgb_callback, 10)

        self.create_subscription(PointCloud2,
                                 "/rgbd_camera/points",
                                 self.pc_callback, 10)

        # TF broadcaster
        self.br = TransformBroadcaster(self)

        self.get_logger().info("✅ Color TF from PointCloud Started")

        # HSV color masks
        self.colors = {
            "red":    [(0,120,70),   (10,255,255)],
            "green":  [(36,50,70),   (89,255,255)],
            "blue":   [(94,80,2),    (126,255,255)],
            "yellow": [(15,120,120), (35,255,255)]
        }

    def pc_callback(self, msg):
        self.pc = msg

    def rgb_callback(self, msg):
        if self.pc is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        height, width, _ = frame.shape

        for name, (lower, upper) in self.colors.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            cx = x + w // 2
            cy = y + h // 2

            # --- READ POINT FROM POINTCLOUD ---
            index = cy * self.pc.width + cx

            # retrieve offsets
            x_offset = next(f.offset for f in self.pc.fields if f.name == 'x')
            y_offset = next(f.offset for f in self.pc.fields if f.name == 'y')
            z_offset = next(f.offset for f in self.pc.fields if f.name == 'z')

            base = index * self.pc.point_step

            import struct
            X = struct.unpack_from('f', self.pc.data, base + x_offset)[0]
            Y = struct.unpack_from('f', self.pc.data, base + y_offset)[0]
            Z = struct.unpack_from('f', self.pc.data, base + z_offset)[0]

            # skip invalids
            if np.isnan(X) or np.isnan(Y) or np.isnan(Z):
                continue

            # Publish TF
            self.publish_tf(name, X, Y, Z)



    def publish_tf(self, name, X, Y, Z):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()

        # point cloud frame
        t.header.frame_id = self.pc.header.frame_id
        t.child_frame_id = f"{name}_block"

        t.transform.translation.x = float(X)
        t.transform.translation.y = float(Y)
        t.transform.translation.z = float(Z)

        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)
        self.get_logger().info(
            f"📍 TF {name} from PointCloud: ({X:.3f}, {Y:.3f}, {Z:.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ColorTFPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
