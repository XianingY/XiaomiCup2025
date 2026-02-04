'''
This demo show the communication interface of MR813 motion control board based on Lcm
- robot_control_cmd_lcmt.py
- file_send_lcmt.py
- Gait_Def_moonwalk.toml
- Gait_Params_moonwalk.toml
- Usergait_List.toml
'''
import lcm
import sys
import time
import toml
import copy
import math
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from file_send_lcmt import file_send_lcmt
from threading import Thread, Lock
from robot_control_response_lcmt import robot_control_response_lcmt
from robot_control import Robot_Ctrl

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
    lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    lcm_usergait = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    usergait_msg = file_send_lcmt()
    msg = robot_control_cmd_lcmt()
    Ctrl = Robot_Ctrl()
    Ctrl.run()
    try:
        msg.mode = 12  # Recovery stand
        msg.gait_id = 0
        msg.life_count += 1  # Command will take effect when life_count update
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(12, 0)
        
        
        print("限高杆开始")

        steps = toml.load("toml/Gait_Params_downwalk.toml")
        full_steps = {'step': [robot_cmd]}
        k = 0
        for i in steps['step']:
            cmd = copy.deepcopy(robot_cmd)
            cmd['duration'] = i['duration']
            if i['type'] == 'usergait':
                cmd['mode'] = 11  # LOCOMOTION
                cmd['gait_id'] = 110  # USERGAIT
                cmd['vel_des'] = i['body_vel_des']
                cmd['rpy_des'] = i['body_pos_des'][0:3]
                cmd['pos_des'] = i['body_pos_des'][3:6]
                cmd['foot_pose'][0:2] = i['landing_pos_des'][0:2]
                cmd['foot_pose'][2:4] = i['landing_pos_des'][3:5]
                cmd['foot_pose'][4:6] = i['landing_pos_des'][6:8]
                cmd['ctrl_point'][0:2] = i['landing_pos_des'][9:11]
                cmd['step_height'][0] = math.ceil(i['step_height'][0] * 1e3) + math.ceil(i['step_height'][1] * 1e3) * 1e3
                cmd['step_height'][1] = math.ceil(i['step_height'][2] * 1e3) + math.ceil(i['step_height'][3] * 1e3) * 1e3
                cmd['acc_des'] = i['weight']
                cmd['value'] = i['use_mpc_traj']
                cmd['contact'] = math.floor(i['landing_gain'] * 1e1)
                cmd['ctrl_point'][2] = i['mu']
            if k == 0:
                full_steps['step'] = [cmd]
            else:
                full_steps['step'].append(cmd)
            k = k + 1
        f = open("toml/Gait_Params_downwalk_full.toml", 'w')
        f.write("# Gait Params\n")
        f.writelines(toml.dumps(full_steps))
        f.close()

        file_obj_gait_def = open("toml/Gait_Def_downwalk.toml", 'r')
        file_obj_gait_params = open("toml/Gait_Params_downwalk_full.toml", 'r')
        usergait_msg.data = file_obj_gait_def.read()
        lcm_usergait.publish("user_gait_file", usergait_msg.encode())
        time.sleep(0.5)
        usergait_msg.data = file_obj_gait_params.read()
        lcm_usergait.publish("user_gait_file", usergait_msg.encode())
        time.sleep(0.1)
        file_obj_gait_def.close()
        file_obj_gait_params.close()

        user_gait_list = open("toml/Usergait_List.toml", 'r')
        steps = toml.load(user_gait_list)
        for step in steps['step']:
            msg.mode = step['mode']
            msg.value = step['value']
            msg.contact = step['contact']
            msg.gait_id = step['gait_id']
            msg.duration = step['duration']
            msg.life_count += 1
            for i in range(3):
                msg.vel_des[i] = step['vel_des'][i]
                msg.rpy_des[i] = step['rpy_des'][i]
                msg.pos_des[i] = step['pos_des'][i]
                msg.acc_des[i] = step['acc_des'][i]
                msg.acc_des[i + 3] = step['acc_des'][i + 3]
                msg.foot_pose[i] = step['foot_pose'][i]
                msg.ctrl_point[i] = step['ctrl_point'][i]
            for i in range(2):
                msg.step_height[i] = step['step_height'][i]
            lcm_cmd.publish("robot_control_cmd", msg.encode())
            time.sleep(0.1)
        for i in range(100):  # 15s Heat beat It is used to maintain the heartbeat when life count is not updated
            lcm_cmd.publish("robot_control_cmd", msg.encode())
            print(f"wait{i}")
            time.sleep(0.2)

        print("限高杆结束")

        msg.mode = 11  # Locomotion
        msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0.1, 0, 0]  # 转向 前 左 左转 速度
        msg.duration = 0  # Zero duration means continuous motion until a new command is used.
        # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.03, 0.03]  # ground clearness of swing leg
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep(1)  # 持续时间
        
        msg.mode = 12  # Recovery stand
        msg.gait_id = 0
        msg.life_count += 1  # Command will take effect when life_count update
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(12, 0)

        Ctrl.perform_line_detection_and_adjustment(msg,bottom_threshold_ratio=0.9)
        print("石板路开始")
        steps = toml.load("toml/Gait_Params_stonewalk.toml")
        full_steps = {'step': [robot_cmd]}
        k = 0
        for i in steps['step']:
            cmd = copy.deepcopy(robot_cmd)
            cmd['duration'] = i['duration']
            if i['type'] == 'usergait':
                cmd['mode'] = 11  # LOCOMOTION
                cmd['gait_id'] = 110  # USERGAIT
                cmd['vel_des'] = i['body_vel_des']
                cmd['rpy_des'] = i['body_pos_des'][0:3]
                cmd['pos_des'] = i['body_pos_des'][3:6]
                cmd['foot_pose'][0:2] = i['landing_pos_des'][0:2]
                cmd['foot_pose'][2:4] = i['landing_pos_des'][3:5]
                cmd['foot_pose'][4:6] = i['landing_pos_des'][6:8]
                cmd['ctrl_point'][0:2] = i['landing_pos_des'][9:11]
                cmd['step_height'][0] = math.ceil(i['step_height'][0] * 1e3) + math.ceil(i['step_height'][1] * 1e3) * 1e3
                cmd['step_height'][1] = math.ceil(i['step_height'][2] * 1e3) + math.ceil(i['step_height'][3] * 1e3) * 1e3
                cmd['acc_des'] = i['weight']
                cmd['value'] = i['use_mpc_traj']
                cmd['contact'] = math.floor(i['landing_gain'] * 1e1)
                cmd['ctrl_point'][2] = i['mu']
            if k == 0:
                full_steps['step'] = [cmd]
            else:
                full_steps['step'].append(cmd)
            k = k + 1
        f = open("toml/Gait_Params_stonewalk_full.toml", 'w')
        f.write("# Gait Params\n")
        f.writelines(toml.dumps(full_steps))
        f.close()

        file_obj_gait_def = open("toml/Gait_Def_stonewalk.toml", 'r')
        file_obj_gait_params = open("toml/Gait_Params_stonewalk_full.toml", 'r')
        usergait_msg.data = file_obj_gait_def.read()
        lcm_usergait.publish("user_gait_file", usergait_msg.encode())
        time.sleep(0.5)
        usergait_msg.data = file_obj_gait_params.read()
        lcm_usergait.publish("user_gait_file", usergait_msg.encode())
        #print(usergait_msg.data)
        time.sleep(0.1)
        file_obj_gait_def.close()
        file_obj_gait_params.close()

        user_gait_list = open("toml/Usergait_List.toml", 'r')
        steps = toml.load(user_gait_list)
        for step in steps['step']:
            msg.mode = step['mode']
            msg.value = step['value']
            msg.contact = step['contact']
            msg.gait_id = step['gait_id']
            msg.duration = step['duration']
            msg.life_count += 1
            for i in range(3):
                msg.vel_des[i] = step['vel_des'][i]
                msg.rpy_des[i] = step['rpy_des'][i]
                msg.pos_des[i] = step['pos_des'][i]
                msg.acc_des[i] = step['acc_des'][i]
                msg.acc_des[i + 3] = step['acc_des'][i + 3]
                msg.foot_pose[i] = step['foot_pose'][i]
                msg.ctrl_point[i] = step['ctrl_point'][i]
            for i in range(2):
                msg.step_height[i] = step['step_height'][i]
            print(f"设置步态：模式为 {msg.mode}，步态 ID 为 {msg.gait_id}，速度为 {msg.vel_des}，步高为 {msg.step_height}，生命计数为 {msg.life_count}")  # 打印步态信息
            lcm_cmd.publish("robot_control_cmd", msg.encode())
            time.sleep(0.1)
        for i in range(135):  # 15s Heat beat It is used to maintain the heartbeat when life count is not updated
            lcm_cmd.publish("robot_control_cmd", msg.encode())
            print(f"wait{i}")
            time.sleep(0.2)
        # msg.mode = 11 # Locomotion
        # msg.gait_id = 27 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        # msg.vel_des = [0.3, 0, 0 ] #转向 前 左 左转 速度
        # msg.duration = 0 # Zero duration means continuous motion until a new command is used.
        #                  # Continuous motion can interrupt non-zero duration interpolation motion
        # msg.step_height = [0.06, 0.06] # ground clearness of swing leg
        # msg.life_count += 1
        # Ctrl.Send_cmd(msg)
        # time.sleep(30) #持续时间

        print("石板路结束")


        msg.mode = 11  # Locomotion
        msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0.1, 0, 0]  # 转向 前 左 左转 速度
        msg.duration = 0  # Zero duration means continuous motion until a new command is used.
        # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.03, 0.03]  # ground clearness of swing leg
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep(1)  # 持续时间

        print("保持站立")
        msg.mode = 12  # Recovery stand
        msg.gait_id = 0
        msg.life_count += 1  # Command will take effect when life_count update
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(12, 0)


    except KeyboardInterrupt:
        msg.mode = 7  # PureDamper before KeyboardInterrupt:
        msg.gait_id = 0
        msg.duration = 0
        msg.life_count += 1
        print(f"设置模式为 {msg.mode}，步态 ID 为 {msg.gait_id}，生命计数为 {msg.life_count}")  # 打印步态信息
        lcm_cmd.publish("robot_control_cmd", msg.encode())
        pass
    sys.exit()


# Main function
if __name__ == '__main__':
    main()
