import time
import copy
import ctypes
from ctypes import sizeof
from rclpy.logging import get_logger

from .sensor_cmd import SensorCmd
from .sensor_para import finger_params, DynamicYddsComTs, DynamicYddsU16Ts

class CapData:
    def __init__(self):
        self.reset()
        
    def reset(self):
        self.sensor_index = 0
        self.channel_cap_data = []
        self.tf = []
        self.tf_dir = []
        self.nf = []
        self.s_prox_cap_data = []
        self.m_prox_cap_data = []

    def init(self, addr, ydds_num, s_prox_num, m_prox_num, cap_channel_num):
        self.sensor_index = addr
        self.channel_cap_data = [0] * cap_channel_num
        self.tf = [0.0] * ydds_num
        self.tf_dir = [0] * ydds_num
        self.nf = [0.0] * ydds_num
        self.s_prox_cap_data = [0] * s_prox_num
        self.m_prox_cap_data = [0] * m_prox_num

    def deinit(self):
        self.reset()

class Finger:
    def __init__(self, pca_idx, ch341):
        self.logger = get_logger(f'finger_{pca_idx}')
        self.sns_cmd = SensorCmd(ch341)
        self.pca_idx = pca_idx
        self.read_data = CapData()
        self.disconnected()

    def disconnected(self):
        self.addr = 0xFF
        self.connect = False
        self.pack_idx = 0
        self.connect_timer = 0
        self.project_para = finger_params[0] # Default
        self.read_data.deinit()

    def connected(self, addr):
        self.addr = addr
        self.connect = True
        self.connect_timer = time.time()
        self.pack_idx = 0
        self.data = [0] * self.project_para.pack_len
        self.read_data.init(addr, 
                            self.project_para.ydds_num,
                            self.project_para.s_prox_num,
                            self.project_para.m_prox_num,
                            self.project_para.sensor_num)

    def check_sensor(self):
        addr_read = self.sns_cmd.get_addr(0)
        # 简化逻辑：只要读到地址就认为连接
        if addr_read == 0: return False

        if not self.sns_cmd.set_sensor_send_type(addr_read, 0):
            self.logger.warn(f"Set Send Type Failed: addr={addr_read}")
        
        if not self.sns_cmd.set_sensor_cap_offset(addr_read, addr_read):
            self.logger.warn(f"Set Cap Offset Failed: addr={addr_read}")

        project_read = self.sns_cmd.get_sensor_project_index(addr_read)
        self.logger.info(f"Detected Project ID: {project_read}")
        
        found = False
        if project_read > 0:
            for pro in finger_params:
                if pro.prg == project_read:
                    self.project_para = copy.deepcopy(pro)
                    self.logger.info(f"Finger Connected: {self.project_para.name}")
                    found = True
                    break
        
        if not found:
            self.logger.warn("Project not found, using default parameters")
            
        self.connected(addr_read)
        return True

    def cap_read(self):
        if not self.connect: return False
        
        rcv_flag = False
        # 重试3次
        for _ in range(3):
            if self.sns_cmd.get_sensor_cap_data(self.addr, self.data):
                if self.data[5] != self.project_para.sensor_num:
                    pass # 通道数不匹配警告

                # 检查包序号
                if self.data[4] != self.pack_idx:
                    self.pack_idx = self.data[4]
                    self.connect_timer = time.time() # 刷新超时时间
                    
                    # 解析逻辑 (简化版)
                    # 1. 原始电容数据
                    cap_byte = self.project_para.cap_byte
                    sens_num = self.project_para.sensor_num
                    
                    for j in range(sens_num):
                        val = 0
                        base = 6 + j * cap_byte
                        if cap_byte == 4:
                            val = int.from_bytes(self.data[base:base+4], 'little')
                        else:
                            val = int.from_bytes(self.data[base:base+3], 'little')
                        self.read_data.channel_cap_data[j] = val

                    # 2. 力数据 (YDDS)
                    ydds_offset = 6 + sens_num * cap_byte
                    ydds_num = self.project_para.ydds_num
                    
                    if self.project_para.ydds_type == 2:
                        sz = sizeof(DynamicYddsComTs)
                        for i in range(ydds_num):
                            start = ydds_offset + i * sz
                            chunk = bytes(self.data[start:start+sz])
                            inst = DynamicYddsComTs.from_buffer_copy(chunk)
                            self.read_data.nf[i] = float(inst.nf)
                            self.read_data.tf[i] = float(inst.tf)
                            self.read_data.tf_dir[i] = int(inst.tfDir)
                            self.read_data.s_prox_cap_data[i] = int(inst.prox)
                            
                    elif self.project_para.ydds_type == 4:
                        sz = sizeof(DynamicYddsU16Ts)
                        for i in range(ydds_num):
                            start = ydds_offset + i * sz
                            chunk = bytes(self.data[start:start+sz])
                            inst = DynamicYddsU16Ts.from_buffer_copy(chunk)
                            self.read_data.nf[i] = inst.nf / 100.0
                            self.read_data.tf[i] = inst.tf / 100.0
                            self.read_data.tf_dir[i] = inst.tfDir
                            
                        # 此类型下 prox 在后面
                        s_prox_off = ydds_offset + ydds_num * sz
                        for i in range(self.project_para.s_prox_num):
                            base = s_prox_off + i * cap_byte
                            self.read_data.s_prox_cap_data[i] = int.from_bytes(self.data[base:base+3], 'little')
                        # ... (在 s_prox 解析循环之后)

                        # 计算 m_prox 的起始偏移量
                        # 偏移量 = YDDS数据结束位置 + 自电容数据长度
                        m_prox_off = s_prox_off + self.project_para.s_prox_num * cap_byte
                        for i in range(self.project_para.m_prox_num):
                            base = m_prox_off + i * cap_byte
                            # 同样使用 int.from_bytes 优雅解析
                            self.read_data.m_prox_cap_data[i] = int.from_bytes(self.data[base:base+3], 'little')
                    rcv_flag = True
                break

        # 超时检查 (2秒)
        if (time.time() - self.connect_timer) > 2.0:
            self.logger.warn(f"Sensor Timeout: addr={self.addr}")
            self.disconnected()
            
        return rcv_flag