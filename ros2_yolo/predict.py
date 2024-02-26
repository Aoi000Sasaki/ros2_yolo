import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, VisionClass, LabelInfo
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
        self.label_publisher = self.create_publisher(LabelInfo, '/ros2_yolo/labels', 10)
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
        for i in range(len(result.boxes)):
            detection = Detection2D()
            detection.header = header
            objectHypothesis = ObjectHypothesisWithPose()
            objectHypothesis.hypothesis.class_id = str(int(result.boxes.cls[i]))
            objectHypothesis.hypothesis.score = float(result.boxes.conf[i])
            detection.results.append(objectHypothesis)
            detection.bbox.center.position.x = float((result.boxes.xyxy[i][2] - result.boxes.xyxy[i][0]) / 2)
            detection.bbox.center.position.y = float((result.boxes.xyxy[i][3] - result.boxes.xyxy[i][1]) / 2)
            detection.bbox.size_x = float(result.boxes.xywh[i][2])
            detection.bbox.size_y = float(result.boxes.xywh[i][3])
            vision_msg.detections.append(detection)

        label_info = LabelInfo()
        label_info.header = header
        for label_id in result.names:
            vision_class = VisionClass()
            vision_class.class_id = label_id
            vision_class.class_name = result.names[label_id]
            label_info.class_map.append(vision_class)

        pred_msg = self.bridge.cv2_to_imgmsg(pred_image, encoding='bgr8')
        pred_msg.header = header
        self.img_publisher.publish(pred_msg)
        self.vmsg_publisher.publish(vision_msg)
        self.label_publisher.publish(label_info)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = YOLOPredictor()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()