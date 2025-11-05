#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory

class PGMPublisher(Node):
    def __init__(self):
        super().__init__('pgm_publisher')
        self.pub = self.create_publisher(Image, '/pgm/image', 10)  # <-- match GUI topic
        self.bridge = CvBridge()

        # Get path to image
        pkg_share = get_package_share_directory('ugv_dashboard_gui')
        self.pgm_path = os.path.join(pkg_share, 'test_images', 'siteA.pgm')
        self.img = cv2.imread(self.pgm_path, cv2.IMREAD_GRAYSCALE)

        if self.img is None:
            self.get_logger().error(f"Could not load image at {self.pgm_path}")
            return

        # Timer to repeatedly publish
        self.timer = self.create_timer(1.0, self.publish_image)  # every 1 second

    def publish_image(self):
        msg = self.bridge.cv2_to_imgmsg(self.img, encoding='mono8')
        self.pub.publish(msg)
        self.get_logger().info(f"Published PGM image from {self.pgm_path}")

def main():
    rclpy.init()
    node = PGMPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
