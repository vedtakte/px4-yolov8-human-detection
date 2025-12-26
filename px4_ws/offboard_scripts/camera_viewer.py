#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()
        # Subscribe to Gazebo camera topic (update topic if different)
        self.subscription = self.create_subscription(
            Image,
            '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Convert ROS Image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Drone Camera Feed", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
