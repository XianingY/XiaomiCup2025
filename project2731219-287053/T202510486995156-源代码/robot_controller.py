import lcm
import time
from threading import Thread, Lock
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt


class RobotController:
    def __init__(self, logger=None):
        # LCM通信设置
        self.lc_send = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")  # 发送通道
        self.lc_recv = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")  # 接收通道

        # 状态变量
        self.lock = Lock()
        self.running = True
        self.logger = logger
        self.last_response = None
        self.cmd_counter = 0

        # 初始化消息结构
        self.cmd_msg = robot_control_cmd_lcmt()
        self._init_default_cmd()

        # 启动接收线程
        self.lc_recv.subscribe("robot_control_response", self._response_handler)
        self.recv_thread = Thread(target=self._recv_loop)
        self.recv_thread.start()

        # 启动心跳线程
        self.heartbeat_thread = Thread(target=self._heartbeat_loop)
        self.heartbeat_thread.start()

    def _init_default_cmd(self):
        """初始化默认命令参数"""
        self.cmd_msg.mode = 0
        self.cmd_msg.gait_id = 0
        self.cmd_msg.life_count = 0
        self.cmd_msg.vel_des = [0.0, 0.0, 0.0]
        self.cmd_msg.pos_des = [0.0, 0.0, 0.0]
        self.cmd_msg.rpy_des = [0.0, 0.0, 0.0]
        self.cmd_msg.duration = 0
        self.cmd_msg.step_height = [0.08, 0.08]

    def _response_handler(self, channel, data):
        """处理控制板响应"""
        try:
            response = robot_control_response_lcmt().decode(data)
            with self.lock:
                # 仅解析实际存在的字段
                self.last_response = {
                    'timestamp': time.time(),
                    'mode': response.mode,
                    'gait_id': response.gait_id,
                    'progress': response.order_process_bar,
                    # 移除了不存在的foot_position和imu_data字段
                }
                if self.logger:
                    self.logger.debug(f"收到响应: mode={response.mode}, progress={response.order_process_bar}%")
        except Exception as e:
            if self.logger:
                self.logger.error(f"响应解析错误: {str(e)}")

    def _recv_loop(self):
        """接收线程主循环"""
        while self.running:
            self.lc_recv.handle_timeout(50)  # 50ms超时

    def _heartbeat_loop(self):
        """心跳线程保持连接"""
        while self.running:
            with self.lock:
                self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 127
                self.lc_send.publish("robot_control_cmd", self.cmd_msg.encode())
            time.sleep(0.1)  # 10Hz心跳

    def send_stand_command(self):
        """发送站立指令"""
        try:
            with self.lock:
                #msg = robot_control_cmd_lcmt()
                msg = self.cmd_msg
                # 必须设置的字段
                msg.mode = 12  # 站立模式
                msg.gait_id = 0
                msg.life_count += 1

                # 运动参数
                msg.vel_des = [0.0, 0.0, 0.0]
                msg.pos_des = [0.0, 0.0, 0.25]  # 站立高度
                msg.rpy_des = [0.0, 0.0, 0.0]
                msg.duration = 0  # 2秒完成动作
                msg.step_height = [0.08, 0.08]

                self.lc_send.publish("robot_control_cmd", msg.encode())

                if self.logger:
                    self.logger.info(
                        f"发送站立指令: mode={msg.mode}, "
                        f"life_count={msg.life_count}, "
                        f"height={msg.pos_des[2]}m"
                    )
            return True
        

        except Exception as e:
            if self.logger:
                self.logger.error(f"指令发送失败: {str(e)}")
            return False
        
    def send_move_command(self):
        """发送站立指令"""
        try:
            with self.lock:
                #msg = robot_control_cmd_lcmt()
                msg = self.cmd_msg
                # 必须设置的字段
                msg.mode = 11  # 站立模式
                msg.gait_id = 27
                msg.life_count += 1

                # 运动参数
                msg.vel_des = [0.5, 0.0, 0.0]
                msg.pos_des = [0.0, 0.0, 0.0]  # 站立高度
                msg.rpy_des = [0.0, 0.0, 0.0]
                msg.duration = 2000  # 2秒完成动作
                msg.step_height = [0.06, 0.06]

                self.lc_send.publish("robot_control_cmd", msg.encode())

                if self.logger:
                    self.logger.info(
                        f"发送站立指令: mode={msg.mode}, "
                        f"life_count={msg.life_count}, "
                    )
            return True
        

        except Exception as e:
            if self.logger:
                self.logger.error(f"指令发送失败: {str(e)}")
            return False

    def get_status(self):
        """获取当前机器人状态"""
        with self.lock:
            if self.last_response:
                return {
                    'mode': self.last_response['mode'],
                    'progress': self.last_response['progress'],
                    'last_update': time.time() - self.last_response['timestamp']
                }
            return {'status': 'no_response'}

    def shutdown(self):
        """安全关闭"""
        self.running = False
        self.recv_thread.join()
        self.heartbeat_thread.join()

        # 发送停止指令
        stop_msg = robot_control_cmd_lcmt()
        stop_msg.mode = 7  # 阻尼模式
        stop_msg.life_count = (self.cmd_counter % 255) + 1
        self.lc_send.publish("robot_control_cmd", stop_msg.encode())

        if self.logger:
            self.logger.info("控制器已安全关闭")