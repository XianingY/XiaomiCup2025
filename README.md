# XiaomiCup2025

2025年小米杯机器人竞赛参赛作品 - 基于 Python、ROS 2 与 LCM 的四足机器人自主导航与视觉识别系统。

## 项目简介

本项目实现了四足机器人在竞赛场地内的全自主运行，集成了视觉感知、路径规划、运动控制与任务执行等核心模块。机器人能够自主巡线、识别场地标识（如绿色箭头、黄灯、斜坡、二维码），并完成指定的搬运与导航任务。

## 核心功能

| 功能模块 | 描述 |
|---------|------|
| **自主巡线** | 基于 OpenCV 的视觉巡线算法，实现稳定的线路跟踪与转向控制 |
| **视觉识别** | 绿色箭头方向识别、斜坡与黄灯检测、二维码扫描与解析 |
| **运动控制** | S 曲线平滑运动、预瞄追踪控制、精准的里程计反馈 |
| **任务执行** | 物块搬运、斜坡通行、动态路径规划与决策 |
| **通讯架构** | ROS 2 节点通信与 LCM 高性能指令传输 |

## 技术栈

- **编程语言**: Python 3
- **机器人框架**: ROS 2 (rclpy)
- **通讯协议**: LCM (Lightweight Communications and Marshaling)
- **计算机视觉**: OpenCV, NumPy, pyzbar
- **传感器**: 摄像头、TOF 传感器 (Time-of-Flight)
- **配置管理**: TOML 格式参数文件

## 目录结构

```
XiaomiCup2025/
├── my/                          # 核心开发目录
│   ├── robot_control.py         # 机器人底层运动控制
│   ├── robot_controller.py      # 高层任务控制器
│   ├── linecontrol.py           # 巡线控制逻辑
│   ├── green_arrow_direction.py # 绿色箭头识别
│   ├── slope_yellowLight.py     # 斜坡与黄灯检测
│   ├── qrread.py                # 二维码读取
│   ├── scurve.py                # S 曲线运动算法
│   ├── pursuit_controller.py    # 预瞄追踪控制器
│   ├── image_subscriber.py      # ROS 2 图像订阅节点
│   ├── odo.py                   # 里程计数据处理
│   ├── toml/                    # 配置文件目录
│   │   ├── Gait_Params_*.toml   # 步态参数配置
│   │   └── Usergait_List.toml   # 用户步态列表
│   └── img/                     # 视觉算法资源图
│
├── project2731219-287053/       # 作品提交包
│   ├── T202510486995156-设计文档.pdf
│   ├── T202510486995156-作品说明.pptx
│   └── 源代码/
│
└── README.md                    # 项目说明文档
```

## 快速开始

### 环境依赖

```bash
# ROS 2 安装 (以 Humble 为例)
sudo apt install ros-humble-desktop

# Python 依赖
pip install opencv-python numpy pyzbar lcm
```

### 运行项目

```bash
# 启动机器人控制节点
python my/robot_control.py

# 或运行完整的竞赛任务
python my/robot_controller.py
```

## 主要模块说明

### 运动控制

- `robot_control.py`: 底层电机控制接口，接收 LCM 指令并执行
- `scurve.py`: 实现 S 形加减速曲线，保证运动平滑
- `pursuit_controller.py`: 预瞄追踪算法，预测路径并提前转向

### 视觉感知

- `green_arrow_direction.py`: 识别场地中的绿色箭头，确定行进方向
- `slope_yellowLight.py`: 检测斜坡特征与黄灯信号
- `qrread.py`: 扫描二维码获取任务信息或场地坐标

### 巡线导航

- `linecontrol.py`: 基于视觉的巡线控制器
- `y_linecontrol.py`: 黄色标识线巡线变体
- `image_subscriber.py`: 订阅摄像头图像流

## 许可证

本项目采用 MIT 许可证开源。
