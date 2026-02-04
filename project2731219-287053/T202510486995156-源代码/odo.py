import lcm
import time
from threading import Thread, Lock

from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
from localization_lcmt import localization_lcmt

# ----------- OdomReceiver: 订阅里程计消息，实时获取yaw角 -----------
class OdomReceiver:
    def __init__(self):
        self.lc_odo = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
        self.odo_msg = localization_lcmt()
        self.new_odo = False
        self.lc_odo.subscribe("global_to_robot", self.odo_callback)

    def odo_callback(self, channel, data):
        self.odo_msg = localization_lcmt().decode(data)
        self.new_odo = True

    def get_yaw(self, timeout=2):
        t0 = time.time()
        while True:
            self.lc_odo.handle()
            if hasattr(self.odo_msg, "rpy") and self.odo_msg.rpy != [0, 0, 0]:
                return self.odo_msg.rpy[2]
            if time.time() - t0 > timeout:
                raise RuntimeError("Timeout waiting odometry!")
            time.sleep(0.05)
            
    def get_yaw(self, timeout=2):
        t0 = time.time()
        while True:
            self.lc_odo.handle()
            if hasattr(self.odo_msg, "rpy") and self.odo_msg.rpy != [0, 0, 0]:
                return self.odo_msg.rpy[2]
            if time.time() - t0 > timeout:
                raise RuntimeError("Timeout waiting odometry!")
            time.sleep(0.05)

    