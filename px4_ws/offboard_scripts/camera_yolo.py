import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

class CameraYOLO(Node):
    def __init__(self):
        super().__init__('camera_yolo_node')
        self.subscription = self.create_subscription(
            Image,
            '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")  # small YOLOv8 model

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(frame)
        annotated = results[0].plot()
        cv2.imshow("YOLO Detection", annotated)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraYOLO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
