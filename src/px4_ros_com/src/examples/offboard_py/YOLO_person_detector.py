#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO
import cv2

class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector_node')

        self.publisher_ = self.create_publisher(String, 'person_position', 10)

        self.model = YOLO('yolov8n.pt')  # asigură-te că e descărcat sau se va descărca automa
        self.cap = cv2.VideoCapture(0)

        self.timer = self.create_timer(0.1, self.detect_person)

    def detect_person(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Camera nu funcționează.')
            return

        results = self.model(frame)[0]

        person_found = False
        direction = "none"

        frame_center = frame.shape[1] // 2

        for box in results.boxes:
            cls = int(box.cls[0].item())
            if cls != 0:
                continue

            person_found = True
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = (x1 + x2) // 2

            if cx < frame_center - 100:
                direction = "left"
            elif cx > frame_center + 100:
                direction = "right"
            else:
                direction = "center"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'Person: {direction}', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            break

        cv2.line(frame, (frame_center, 0), (frame_center, frame.shape[0]), (255, 0, 0), 2)

        cv2.imshow('YOLOv8 Person Detection', frame)
        cv2.waitKey(1)

        msg = String()
        msg.data = f"person:{direction}" if person_found else "noperson"
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
