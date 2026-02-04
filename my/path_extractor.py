import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PathExtractor(Node):
    def __init__(self):
        super().__init__('path_extractor')
        self.subscription = self.create_subscription(
            Image,
            '/image_rgb',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

        # 取图像中下半部分
        roi = binary[binary.shape[0]//2:, :]
        
        # 计算每一行白色区域的中心
        path_points = []
        for y in range(roi.shape[0]):
            row = roi[y]
            white_pixels = np.where(row == 255)[0]
            if len(white_pixels) > 0:
                center_x = int(np.mean(white_pixels))
                path_points.append((center_x, y))

        # 发布路径点（下一步可用于控制）
        if len(path_points) > 5:
            np.save('/tmp/waypoints.npy', np.array(path_points))  # 用文件中转，简化调试
