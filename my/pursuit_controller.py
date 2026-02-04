from geometry_msgs.msg import Twist
import numpy as np

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pursuit_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.lookahead_distance = 30

    def control_loop(self):
        try:
            waypoints = np.load('/tmp/waypoints.npy')  # 从共享路径读取
        except:
            return

        target = None
        for point in waypoints:
            if point[1] > self.lookahead_distance:
                target = point
                break

        if target is not None:
            # 转换为机器人坐标系角度（假设中心为图像中心）
            cx = 160  # 假设图像宽为 320
            dx = target[0] - cx
            angle = np.arctan2(dx, self.lookahead_distance)

            # 构造 Twist 消息
            cmd = Twist()
            cmd.linear.x = 0.2
            cmd.angular.z = -float(angle) * 0.01
            self.publisher.publish(cmd)
