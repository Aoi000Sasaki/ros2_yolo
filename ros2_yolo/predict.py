import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO
import torch

class YOLOPredictor(Node):
    def __init__(self):
        super().__init__('predictor')
        self.ws_path = '/home/user/ws/src'
        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.image_callback,
            10)
        self.box_publisher = self.create_publisher(Float32MultiArray, '/ros2_yolo/box', 10)
        self.img_publisher = self.create_publisher(Image, '/ros2_yolo/pred', 10)
        self.bridge = CvBridge()
        self.yolo_model = YOLO(self.ws_path + '/ros2_yolo/weights/yolov8n.pt')

    def image_callback(self, msg):
        timestamp = msg.header.stamp
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # yolo model
        results = self.yolo_model(cv_image)

        pred_image = results[0].plot()
        cv2.imwrite('/ros2_yolo/image/image.jpg', cv_image)
        cv2.imwrite('/ros2_yolo/image/pred.jpg', pred_image)

        reliable_box_array = self.filter_prediction(results)
        # Float32MultiArrayはheaderを持たないため，リストの先頭にタイムスタンプを追加
        reliable_box_array.insert(0, float(timestamp.sec))
        reliable_box_array.insert(1, float(timestamp.nanosec))

        # 信頼度の高いボックスの座標を配信
        box_msg = Float32MultiArray()
        box_msg.data = reliable_box_array
        self.box_publisher.publish(box_msg)

        # 予測結果の画像を配信
        pred_msg = self.bridge.cv2_to_imgmsg(pred_image, encoding='bgr8')
        # タイムスタンプを観測画像のものに合わせる
        pred_msg.header.stamp = timestamp
        self.img_publisher.publish(pred_msg)

    def filter_prediction(self, results):
        # 検出結果
        # 0~3: ボックスの座標(xmin, ymin, xmax, ymax)
        # 4: 信頼度
        # 5: クラス値
        box_datas = results[0].boxes.data

        # 検出対象のクラスのクラス値（ヒトは0）
        tgt_cls = 0
        threshold = 1e-6
        # 検出対象のクラスのデータのみを抽出
        # クラス値の一致（閾値内）で判断している
        tgt_boxes = [box_data for box_data in box_datas if abs(box_data[5] - tgt_cls) <= threshold]
        # 信頼度順で出力されるため，先頭が最も信頼度が高い
        reliable_box = tgt_boxes[0] if tgt_boxes else None

        # リストに変換
        reliable_box_array = reliable_box.tolist() if reliable_box is not None else []
        print(reliable_box_array)

        return reliable_box_array


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = YOLOPredictor()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()