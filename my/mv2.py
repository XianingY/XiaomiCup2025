import sys
import os
import time
from threading import Thread, Lock
import cv2
import numpy as np
import lcm
import toml
import copy
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from green_arrow_direction import ArrowAction
from tofTest3 import RobotQRRunner
from robot_control import Robot_Ctrl
from ehcd_rbctr import EnhancedRobotCtrl
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt
from odo import OdomReceiver
from image_subscriber import ImageSubscriber
from image_subscriber1 import ImageSubscriber1
from file_send_lcmt import file_send_lcmt
from yellowLight_slope2 import YellowLight_slope
from slope_yellowLight_cyc import Slope_YellowLight
from stone_down import Stone_Down
from down_stone import Down_Stone
from scurve import Scurve

robot_cmd = {
   'mode': 0, 'gait_id': 0, 'contact': 0, 'life_count': 0,
   'vel_des': [0.0, 0.0, 0.0],
    'rpy_des': [0.0, 0.0, 0.0],
    'pos_des': [0.0, 0.0, 0.0],
    'acc_des': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ctrl_point': [0.0, 0.0, 0.0],
    'foot_pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
   'step_height': [0.0, 0.0],
    'value': 0,  'duration': 0
    }

def main():
    try:
        print("启动")
        a_runner = A_Scrue()
        print("A")
        slope_runner=Slope()
        print("S")
        b_stone_runner=B_Stone()
        print("BSTONE")


        aCode , direction = a_runner.aArea(code="A-1")
        #print(direction)
        a_runner.scrue()
        #direction="left"
        slope_runner.arrow(direction)

        if direction == "left":
            #石板路-限高杆
            b_stone_runner.stone_down()
            #B区
            a_runner.bArea(direction,code="B-1")
            #黄灯上下坡
            b_stone_runner.yellowLight_slope()
            #上下坡到S弯的衔接
            b_stone_runner.slopeToScuve()

        else:
            #上下坡黄灯
            b_stone_runner.slope_yellowLight()
            #B区
            a_runner.bArea(direction,code="B-1")
            #限高杆-石板路
            b_stone_runner.down_stone()

            #石板路到S弯的衔接
            b_stone_runner.stoneToScuve()

        a_runner.scrue_back()

        a_runner.back_to_station("A-1")

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"错误：{e}")
    finally:
        sys.exit()



class A_Scrue:
    def aArea(self,code="A-1"):
        print("aArea")
        # Ctrl = EnhancedRobotCtrl()
        # Ctrl.run()
        # odo = OdomReceiver()

        # Ctrl.quit()
        qr_runner = RobotQRRunner()
        aCode = qr_runner.qrReadA(defult_code=code)
        return aCode

    def scrue(self):
        print("S弯方法")
        scurve = Scurve()
        scurve.scurve()

    def scrue_back(self):
        print("S弯返回方法")
        scurve = Scurve()
        scurve.scurve_return()


    def bArea(self,direction,code="B-1"):
        print("bArea")
        qr_runner = RobotQRRunner()
        qr_runner.qrReadB(direction,defult_code=code)

    #最后结尾处的行为
    def back_to_station(self,aCode):
        print("a_back")
        # Ctrl = EnhancedRobotCtrl()
        # Ctrl.run()
        # odo = OdomReceiver()

        # Ctrl.quit()
        qr_runner = RobotQRRunner()
        aCode = qr_runner.qrReadBack(aCode)
        return aCode


class Slope:
    def arrow(self,direction):
        arrow_action = RobotQRRunner()
        arrow_action.arrow_Read(direction)

    #上坡前位置微调
    def turnBeforeSlope(self):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()

        try:
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count =Ctrl.get_next_life_count()  # Command will take effect when life_count update
            print(msg)  # 打印步态信息
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            Ctrl.turn_to_direction(270)

            msg.mode = 7  # PureDamper
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(7, 0)

        finally:
            Ctrl.quit()


    def move(self):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()
        msg = robot_control_cmd_lcmt()

        try:
            msg.mode = 12  # Recovery stand
            msg.gait_id = 0
            msg.life_count =Ctrl.get_next_life_count()  # Command will take effect when life_count update
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(12, 0)

            msg.mode = 11  # Locomotion
            msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            msg.vel_des = [0.1, 0, 0]  # 转向 前 左 左转 速度
            msg.duration = 0  # Zero duration means continuous motion until a new command is used.
            # Continuous motion can interrupt non-zero duration interpolation motion
            msg.step_height = [0.06, 0.00]  # ground clearness of swing leg
            msg.life_count =Ctrl.get_next_life_count()
            Ctrl.Send_cmd(msg)
            print(msg.gait_id)
            time.sleep(35)  # 持续时间

            msg.mode = 7  # PureDamper
            msg.gait_id = 0
            msg.life_count += 1
            Ctrl.Send_cmd(msg)
            Ctrl.Wait_finish(7, 0)

        finally:
            Ctrl.quit()


class B_Stone:

    #黄灯
    def yellowLight_slope(self):
        print("黄灯_上下坡")
        yellow=YellowLight_slope()
        yellow.yellowLight_slope()

    def slope_yellowLight(self):
        print("上下坡_黄灯")
        slope=Slope_YellowLight()
        slope.slope_yellowLight()

    def stone_down(self):
        print("石板路_限高杆")
        stone=Stone_Down()
        stone.stone_down()

    def down_stone(self):
        print("限高杆_石板路")
        down=Down_Stone()
        down.down_stone()


    def stoneToScuve(self):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()

        try:
            Ctrl.send_move_command(12,0,[0, 0, 0],duration = 0,step_height = [0.06,0.06])
            Ctrl.Wait_finish(12, 0)

           # Ctrl.turn_to_direction_old(0)
           # Ctrl.send_move_command(11,27,[0,0,0.5])
           # time.sleep(4)

            Ctrl.send_move_command(11,27,[0, 0.5, 0],duration = 0,step_height = [0.06,0.06])
            # Ctrl.Wait_finish(11, 27)
            time.sleep(3)

           # Ctrl.turn_to_direction_old(90)

            Ctrl.send_move_command(11,27,[0.5, 0, 0],duration = 0,step_height = [0.06,0.06])
            # Ctrl.Wait_finish(11, 27)
            time.sleep(1)

            Ctrl.send_move_command(12,0,[0, 0, 0],duration = 0,step_height = [0.06,0.06])
            Ctrl.Wait_finish(12, 0)

            # Ctrl.send_move_command(7,0,[0, 0, 0])
            # Ctrl.Wait_finish(7, 0)

        finally:
            Ctrl.quit()

    def turn_to_direction(self, direction):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()

        try:
            Ctrl.send_move_command(12,0,[0, 0, 0],duration = 0,step_height = [0.06,0.06])
            time.sleep(1)

            Ctrl.turn_to_direction(direction)
            Ctrl.Wait_finish(11,27)

        finally:
            Ctrl.quit()


    def slopeToScuve(self):
        Ctrl = EnhancedRobotCtrl()
        Ctrl.run()

        try:
            Ctrl.send_move_command(12,0,[0, 0, 0],duration = 0,step_height = [0.06,0.06])
            Ctrl.Wait_finish(12, 0)

           # Ctrl.turn_to_direction_old(180)
           # time.sleep(2)

            Ctrl.send_move_command(11,27,[0,-0.5, 0],duration = 0,step_height = [0.06,0.06])
            # Ctrl.Wait_finish(11, 27)
            time.sleep(4.5)

           # Ctrl.turn_to_direction_old(90)

            Ctrl.send_move_command(11,27,[0.5, 0, 0],duration = 0,step_height = [0.06,0.06])
            # Ctrl.Wait_finish(11, 27)
            time.sleep(1)

            Ctrl.send_move_command(12,0,[0, 0, 0],duration = 0,step_height = [0.06,0.06])
            Ctrl.Wait_finish(12, 0)

            # Ctrl.send_move_command(7,0,[0, 0, 0])
            # Ctrl.Wait_finish(7, 0)

        finally:
            Ctrl.quit()

# Main function
if __name__ == '__main__':
    main()
