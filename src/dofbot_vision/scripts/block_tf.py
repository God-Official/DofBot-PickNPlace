#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

import struct
import tf_transformations


class ColorTFPointCloud(Node):
    def __init__(self):
        super().__init__('color_tf_from_pointcloud')

        self.bridge = CvBridge()
        self.pc = None

        # Subscribers
        self.create_subscription(Image,
                                 "/camera/camera/color/image_raw",
                                 self.rgb_callback, 10)
        self.create_subscription(PointCloud2,
                                 "/camera/custom_pointcloud",
                                 self.pc_callback, 10)

        # TF tools
        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("✅ Color TF from PointCloud Started")

        # HSV masks
        self.colors = {
            "red":    [(0,197,98),   (10,255,178)],
            "green":  [(36,50,70),   (89,255,255)],
            "blue":   [(100,215,82),    (120,255,162)],
            "yellow": [(15,120,120), (35,255,255)]
        }

    def pc_callback(self, msg):
        self.pc = msg

    def rgb_callback(self, msg):
        if self.pc is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for name, (lower, upper) in self.colors.items():

            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                continue

            # Get center pixel of largest contour
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            cx = x + w // 2
            cy = y + h // 2

            index = cy * self.pc.width + cx
            base = index * self.pc.point_step

            # Field offsets
            x_offset = next(f.offset for f in self.pc.fields if f.name == 'x')
            y_offset = next(f.offset for f in self.pc.fields if f.name == 'y')
            z_offset = next(f.offset for f in self.pc.fields if f.name == 'z')

            # Read XYZ from point cloud
            X = struct.unpack_from('f', self.pc.data, base + x_offset)[0]
            Y = struct.unpack_from('f', self.pc.data, base + y_offset)[0]
            Z = struct.unpack_from('f', self.pc.data, base + z_offset)[0]

            if np.isnan(X) or np.isnan(Y) or np.isnan(Z):
                continue

            # --- TF from camera optical frame ---
            self.publish_camera_tf(name, X, Y, Z)

            # --- Transform point to base_link frame ---
            self.publish_base_tf(name, X, Y, Z)


    # --------------------------------------------------------
    # camera_optical -> block TF
    # --------------------------------------------------------
    def publish_camera_tf(self, name, X, Y, Z):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()

        t.header.frame_id = self.pc.header.frame_id
        t.child_frame_id = f"{name}_block"

        t.transform.translation.x = float(X)
        t.transform.translation.y = float(Y)
        t.transform.translation.z = float(Z)
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)


    # --------------------------------------------------------
    # base_link -> block_base TF (CORRECT rotation+translation)
    # --------------------------------------------------------
    def publish_base_tf(self, name, X, Y, Z):
        try:
            tf_cam = self.tf_buffer.lookup_transform(
                "base_link",
                self.pc.header.frame_id,         # camera optical link
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"❗ TF lookup failed: {e}")
            return

        # Translation of the camera in base_link frame
        tx = tf_cam.transform.translation.x
        ty = tf_cam.transform.translation.y
        tz = tf_cam.transform.translation.z

        # Rotation quaternion
        qx = tf_cam.transform.rotation.x
        qy = tf_cam.transform.rotation.y
        qz = tf_cam.transform.rotation.z
        qw = tf_cam.transform.rotation.w

        # Build transformation matrix
        T = tf_transformations.quaternion_matrix([qx, qy, qz, qw])
        T[0, 3] = tx
        T[1, 3] = ty
        T[2, 3] = tz

        # Transform point from camera frame → base_link frame
        cam_point = [X, Y, Z, 1.0]
        base_point = T.dot(cam_point)

        bx, by, bz = base_point[0], base_point[1], base_point[2]

        # Publish TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()

        t.header.frame_id = "base_link"
        t.child_frame_id = f"{name}_block_base"

        t.transform.translation.x = float(bx)
        t.transform.translation.y = float(by)
        t.transform.translation.z = float(bz)
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)

        self.get_logger().info(
            f"📌 base_link → {name}_block_base: ({bx:.3f}, {by:.3f}, {bz:.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ColorTFPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
