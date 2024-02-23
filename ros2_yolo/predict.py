import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from vison_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO
import torch

class YOLOPredictor(Node):
    def __init__(self):
        super().__init__('predictor')
        self.ws_path = '/home/user/ws/src'
        self.subscription = self.create_subscription(Image, '/color/image_raw', self.image_callback, 1)
        self.vmsg_publisher = self.create_publisher(Detection2DArray, '/ros2_yolo/detection', 10)
        self.img_publisher = self.create_publisher(Image, '/ros2_yolo/pred', 10)
        self.bridge = CvBridge()
        self.yolo_model = YOLO(self.ws_path + '/ros2_yolo/weights/yolov8n.pt')

    def image_callback(self, msg):
        header = msg.header
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # yolo model
        results = self.yolo_model(cv_image)

        # save image
        pred_image = results[0].plot()
        cv2.imwrite(self.ws_path + '/ros2_yolo/image/image.jpg', cv_image)
        cv2.imwrite(self.ws_path + '/ros2_yolo/image/pred.jpg', pred_image)

        vision_msg = Detection2DArray()
        vision_msg.header = header

        # input single image -> len(results) = 1
        result = results[0]
        for box in result.boxes:
            detection = Detection2D()
            detection.header = header
            objectHypothesis = ObjectHypothesisWithPose()
            objectHypothesis.id = box[5]
            objectHypothesis.score = box[4]
            detection.results.append(objectHypothesis)
            detection.bbox.center.x = (box[2] - box[0]) / 2
            detection.bbox.center.y = (box[3] - box[1]) / 2
            detection.bbox.size_x = box[2] - box[0]
            detection.bbox.size_y = box[3] - box[1]
            # detection.source_img = msg # msg became too large
            vision_msg.detections.append(detection)

        pred_msg = self.bridge.cv2_to_imgmsg(pred_image, encoding='bgr8')
        pred_msg.header.stamp = timestamp
        self.img_publisher.publish(pred_msg)

        self.vmsg_publisher.publish(vision_msg)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = YOLOPredictor()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()