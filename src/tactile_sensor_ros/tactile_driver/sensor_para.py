from typing import List, Optional
from ctypes import Structure, c_float, c_uint32, c_uint16

class DynamicYddsComTs(Structure):
    _pack_ = 1
    _fields_ = [
        ("nf", c_float),
        ("nfCap", c_uint32),
        ("tf", c_float),
        ("tfCap", c_uint32),
        ("tfDir", c_uint16),
        ("prox", c_uint32),
    ]

class DynamicYddsU16Ts(Structure):
    _pack_ = 1
    _fields_ = [
        ("nf", c_uint16),
        ("tf", c_uint16),
        ("tfDir", c_uint16),
    ]

class FingerHeatMap:
    def __init__(self, rows: int, cols: int, file_path: str, cap_count: int, cap_indices: List[int]):
        self.rows = rows
        self.cols = cols
        self.file_path = file_path  # ROS 2 中可能需要修正此路径
        self.cap_count = cap_count
        self.cap_indices = cap_indices

class FingerParamTS:
    def __init__(self, prg: int, pack_len: int, sensor_num: int,
                 touch_num: int, ydds_num: int, s_prox_num: int,
                 m_prox_num: int, cap_byte: int, ydds_type: int,
                 had_err: int, cali_num: int, name: str,
                 display_type_para: str,
                 p_heat_map: Optional[List[FingerHeatMap]]):
        self.prg = prg
        self.pack_len = pack_len
        self.sensor_num = sensor_num
        self.touch_num = touch_num
        self.ydds_num = ydds_num
        self.s_prox_num = s_prox_num
        self.m_prox_num = m_prox_num
        self.cap_byte = cap_byte
        self.ydds_type = ydds_type
        self.had_err = had_err
        self.cali_num = cali_num
        self.name = name
        self.display_type_para = display_type_para
        self.p_heat_map = p_heat_map

# 模拟数据，暂不加载真实的 .dat 文件以免路径错误
finger2_power_cap_index = [
    FingerHeatMap(16, 8, "TS-F-A/heatMapPara16_8.dat", 7, [0]*16)
]
finger17_power_cap_index = [
    FingerHeatMap(6, 7, "TS-T-A/weight6X7X6.dat", 6, [0]*16),
    FingerHeatMap(6, 7, "TS-T-A/weight6X7X6.dat", 6, [0]*16)
]
finger27_power_cap_index = [
    FingerHeatMap(16, 8, "TS-F-A/heatMapPara16_8.dat", 7, [0]*16)
]

finger_params = [
    FingerParamTS(2, 62, 8, 7, 1, 1, 0, 4, 2, 0, 22, "通用手指", "TypeA", finger2_power_cap_index),
    FingerParamTS(17, 78, 16, 13, 2, 2, 1, 3, 4, 1, 22, "两指-大包", "TypeB", finger17_power_cap_index),
    FingerParamTS(27, 66, 16, 15, 1, 1, 0, 3, 4, 1, 22, "通用点阵", "TypeB", finger27_power_cap_index),
    FingerParamTS(44, 60, 14, 13, 1, 1, 0, 3, 4, 1, 22, "通用点阵", "TypeB", finger27_power_cap_index),
    FingerParamTS(50, 36, 6, 5, 1, 1, 0, 3, 4, 1, 22, "通用大拇指", "TypeB", finger27_power_cap_index),
    FingerParamTS(54, 36, 6, 5, 1, 1, 0, 3, 4, 1, 22, "通用小拇指", "TypeB", finger27_power_cap_index),
    FingerParamTS(52, 66, 12, 9, 2, 2, 1, 3, 4, 1, 22, "通用中指", "TypeB", finger27_power_cap_index),
]