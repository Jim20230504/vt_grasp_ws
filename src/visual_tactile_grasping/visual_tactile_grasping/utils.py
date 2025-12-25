#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from enum import Enum, auto

class SystemState(Enum):
    """系统状态机枚举"""
    IDLE = auto()               # 空闲状态，等待初始化
    INITIALIZING = auto()       # 正在初始化硬件
    MOVING_TO_OBSERVE = auto()  # 移动到观察点
    DETECTING = auto()          # 视觉识别中
    PLANNING_APPROACH = auto()  # 规划抓取路径
    APPROACHING = auto()        # 执行移动到抓取上方
    TACTILE_GRASPING = auto()   # 执行触觉自适应抓取 (核心逻辑)
    LIFTING = auto()            # 抓取后抬起
    PLACING = auto()            # 放置物品
    ERROR = auto()              # 故障状态

class GripperState(Enum):
    """夹爪/触觉流程子状态，参考 capGrasp.py"""
    INIT = 0
    OPEN = 1
    WAIT_OPEN = 2
    CALIBRATION = 3
    CLOSING_UNTIL_TOUCH = 4
    CHECKING_FIRST_TOUCH = 5
    ADJUSTING_FOR_BOTH_TOUCH = 6
    GENTLE_CLAMPING = 7
    SLIP_DETECTION = 8
    RELEASE = 9
    DONE = 10
    GRIPPER_LENGTH = 0.30

    # 抓取时，指尖离桌面的高度
    GRASP_HEIGHT_OFFSET = 0.01

# 机械臂预设姿态 (关节角度，单位：弧度，需根据实际情况修改)
OBSERVATION_JOINT_POSE = [0.0, -0.5, 1.57, -1.57, 0.0, 0.0] 
DROP_OFF_JOINT_POSE = [1.57, -0.5, 1.0, -1.57, 0.0, 0.0]

# 相机话题配置
TOPIC_COLOR = '/camera/color/image_raw'
TOPIC_DEPTH = '/camera/aligned_depth_to_color/image_raw'
TOPIC_INFO = '/camera/color/camera_info'