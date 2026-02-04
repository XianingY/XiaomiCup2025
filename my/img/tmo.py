def turn_to_direction(self, direction, tolerance=0.03, max_wait=12, tag=0):
        """
        转向到指定方向

​        Args:
​            direction: 目标方向，可以是：
                    - 数字(度数): 0, 90, 180, 270, 360等
                                        - 字符串: "前"(0°), "右"(90°), "后"(180°), "左"(270°)
                        tolerance: 角度容差(弧度)，默认1度
                        max_wait: 最大等待时间(秒)
                """
        # 将方向转换为目标yaw角度(弧度)
​        if isinstance(direction, str):
​            direction_map = {
​                "前": 0, "右": -90, "后": 180, "左": 270,
​                "north": 0, "east": 90, "south": 180, "west": 270,
​                "n": 0, "e": 90, "s": 180, "w": 270
​            }
​            if direction in direction_map:
​                target_degrees = direction_map[direction]
​            else:
​                raise ValueError(f"未知的方向字符串: {direction}")
​        else:
​            target_degrees = float(direction)

        # 将角度规范化到0-360范围
​        target_degrees = target_degrees % 360
​        now_yaw = self.odo.get_yaw()

        # 转换为弧度
​        if target_degrees == 0:
​            target_yaw = 0  # 前方
​        elif target_degrees == 90:
​            target_yaw = -math.pi/2  # 右方 (-1.5708)
​        elif target_degrees == 180:
​            if now_yaw < 0:
​                target_yaw = -math.pi
​            else:
​                target_yaw = math.pi  # 后方 (3.1416 或 -3.1416)
​        elif target_degrees == 270:
​            target_yaw = math.pi/2  # 左方 (1.5708)
​        else:
​            target_yaw = -self.degrees_to_radians(target_degrees)
​            target_yaw = self.normalize_angle(target_yaw)

​        print(f"目标方向: {direction} -> {target_yaw:.3f} 弧度")

​        current_yaw = self.odo.get_yaw()
​        if self.turn_to_yaw(target_yaw, tolerance, max_wait) or abs(self.normalize_angle(self.odo.get_yaw() - target_yaw)) < tolerance:
​            print(f"一次成功，{current_yaw:.3f} 弧度")
​            return True
​        elif tag == 1:
​            return True
​        elif abs(self.normalize_angle(self.odo.get_yaw() - target_yaw)) < 0.2:
​            return True
​        else:
​            print(f"一次失败，{self.odo.get_yaw():.3f} 弧度")
​            count = 0
​            while not self.turn_to_yaw(target_yaw, tolerance, max_wait) and count < 2:
​                if abs(self.normalize_angle(self.odo.get_yaw() - target_yaw)) < tolerance:
​                    print(f"******重新转动之后,由于延迟最后记录的角度不符合,但是实际是符合的")
​                    break
​                count += 1
​                print(f"角度不精确，当前角度,{self.odo.get_yaw():.3f}")

​    def turn_to_yaw(self, target_yaw, tolerance=0.03, max_wait=12):
​        """
​        转向到指定的绝对yaw角度

​        Args:
​            target_yaw: 目标yaw角度(弧度)，绝对角度
​            tolerance: 角度容差(弧度)
​            max_wait: 最大等待时间(秒)
​        """
​        print(f"开始转向到目标角度: {target_yaw:.3f} 弧度")

​        try:
​            current_yaw = self.odo.get_yaw()
​            print(f"当前yaw角度: {current_yaw:.3f} 弧度")
​        except Exception as e:
​            print(f"无法获取当前角度: {e}")
​            return False

        # 计算最短路径的角度差
​        angle_diff = self.normalize_angle(target_yaw - current_yaw)
​        print(f"需要转动角度: {angle_diff:.3f} 弧度 ({angle_diff*180/math.pi:.1f}度) ({'左转' if angle_diff > 0 else '右转'})")

        # 如果角度差很小，直接返回
​        if abs(angle_diff) < tolerance:
​            print("已经在目标角度附近，无需转向")
​            return True

        # 发送转向指令
​        turn_msg = robot_control_cmd_lcmt()
​        turn_msg.mode = 11
​        turn_msg.gait_id = 26
​        turn_msg.vel_des = [0, 0, 0.5 if angle_diff > 0 else -0.5]
​        turn_msg.duration = int(2470 * abs(angle_diff))
​        turn_msg.step_height = [0.06, 0.06]
​        turn_msg.life_count = self.get_next_life_count()

​        self.Send_cmd(turn_msg)
​        print(f"转向指令已下发，duration: {turn_msg.duration}, life_count: {turn_msg.life_count}")

        # 轮询yaw，直到接近目标
​        t_start = time.time()
​        success = False
​        while True:
​            try:
​                now_yaw = self.odo.get_yaw()
                # 计算当前角度与目标角度的最短距离
​                current_err = abs(self.normalize_angle(now_yaw - target_yaw))
​                print(f"当前yaw: {now_yaw:.3f}，距离目标: {current_err:.3f}")

​                if current_err < tolerance + 0.005:
​                    print("转向完成")
​                    stop_msg = robot_control_cmd_lcmt()
​                    stop_msg.mode = 11
​                    stop_msg.gait_id = 26
​                    stop_msg.vel_des = [0, 0, 0.0]  # 停止转动
​                    stop_msg.duration = 0
​                    stop_msg.life_count = self.get_next_life_count()
​                    self.Send_cmd(stop_msg)
​                    success = True
​                    break
​                if time.time() - t_start > max_wait:
​                    print("转向超时，未到目标角度")
​                    break
​            except Exception as e:
​                print(f"获取角度失败: {e}")
​                if time.time() - t_start > max_wait:
​                    break
​            time.sleep(0.15)

        # 转向完成后，短暂停顿让机器人稳定
​        time.sleep(0.5)
​        print("转向流程结束")
​        return success
