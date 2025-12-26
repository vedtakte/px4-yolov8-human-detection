import gz
import cv2
import numpy as np
from gz.transport13 import Node
from gz.msgs.image_pb2 import Image
from ultralytics import YOLO

# Load YOLOv8 model once
model = YOLO("yolov8n.pt")  # you can download this with: pip install ultralytics

def image_callback(msg: Image):
    # Convert Gazebo Image message to numpy array
    img_data = np.frombuffer(msg.data, dtype=np.uint8)
    img = img_data.reshape((msg.height, msg.width, 3))  # assuming RGB8

    # Run YOLO inference
    results = model(img)

    # Plot detections on the frame
    annotated = results[0].plot()

    # Show in OpenCV window
    cv2.imshow("YOLO Human Detection", annotated)
    cv2.waitKey(1)

def main():
    node = Node()
    topic = "/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image"
    node.subscribe(topic, Image, image_callback)

    print(f"-- Subscribed to {topic}")
    while True:
        pass  # Keep node alive

if __name__ == "__main__":
    main()
