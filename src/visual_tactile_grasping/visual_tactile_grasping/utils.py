from enum import Enum, auto

class SystemState(Enum):
    IDLE = auto()
    INITIALIZING = auto()
    MOVING_TO_OBSERVE = auto()
    DETECTING_WAIT = auto() # 新增
    DETECTING = auto()
    PLANNING_APPROACH = auto()
    APPROACHING = auto()
    TACTILE_GRASPING = auto()
    LIFTING = auto()
    PLACING = auto()
    ERROR = auto()

class GripperState(Enum):
    IDLE = auto()
    INIT = auto()
    OPEN = auto()
    WAIT_OPEN = auto()
    CALIBRATION = auto()
    CLOSING_UNTIL_TOUCH = auto()
    ADJUSTING_FOR_BOTH_TOUCH = auto()
    GENTLE_CLAMPING = auto()
    SLIP_DETECTION = auto()
    RELEASE = auto()


TOPIC_COLOR = '/camera/camera/color/image_raw'
TOPIC_DEPTH = '/camera/camera/aligned_depth_to_color/image_raw'
TOPIC_INFO = '/camera/camera/color/camera_info'

# 预设姿态 (弧度)
# OBSERVATION_JOINT_POSE = [108, 9, 76, 143, -103, 12]
OBSERVATION_JOINT_POSE = [1.884956, 0.157080, 1.326450, 2.495821, -1.797689, 0.209440]

# 放置区域姿态 (根据实际情况调整)
DROP_OFF_JOINT_POSE = [1.57, -0.5, 1.2, 0.0, 1.0, 0.0]

# 指尖目标Z = 法兰Z - 夹爪长度

# GRIPPER_LENGTH = 0.22