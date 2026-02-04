# -*- coding: utf-8 -*-
import lcm
import sys
import os
import time
from threading import Thread, Lock

from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
from localization_lcmt import localization_lcmt

import toml
import copy
import math
import cv2
import numpy as np
from scipy.optimize import minimize
from file_send_lcmt import file_send_lcmt
from middle_adj_linecontrol import LineControl

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor

class MiddleAdj:
    def middleAdj(self):
        try:
            # 初始化控制器
            Ctrl = LineControl()
            Ctrl.run()
            msg = robot_control_cmd_lcmt()

            # 初始姿态调整 - 保持站立
            msg.mode = 12  # Recovery stand模式（站立）
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            Ctrl.m_perform_line_detection_and_adjustment(msg)

            msg.mode = 7  # PureDamper
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(7, 0)


        except KeyboardInterrupt:
            print("\n用户中断程序")
        except Exception as e:
            print(f"发生错误: {str(e)}")
        finally:
            if 'Ctrl' in locals():
                Ctrl.quit()
def main():
    # 执行scurve动作
    middleAdj = MiddleAdj()
    print("s弯动作")
    #scurve.scurve()

if __name__ == '__main__':
    main()

