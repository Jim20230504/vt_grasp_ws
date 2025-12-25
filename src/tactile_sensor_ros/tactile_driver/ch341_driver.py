import time
import os
import sys
import ctypes
from ctypes import c_byte, c_ubyte, c_long  # 引入 c_ubyte
import glob
import rclpy
from rclpy.logging import get_logger
from ament_index_python.packages import get_package_share_directory

# 根据系统类型导入对应的库加载器
if os.name == 'nt':  # Windows
    from ctypes import windll
else:  # Linux
    from ctypes import cdll

class Ch341Driver:
    # 接口固定宏
    _mCH341A_CMD_I2C_STREAM = 0xAA
    _mCH341A_CMD_I2C_STM_STA = 0x74
    _mCH341A_CMD_I2C_STM_STO = 0x75
    _mCH341A_CMD_I2C_STM_OUT = 0x80
    _mCH341A_CMD_I2C_STM_IN = 0xC0
    _mCH341A_CMD_I2C_STM_MAX = 63
    _mCH341A_CMD_I2C_STM_SET = 0x60
    _mCH341A_CMD_I2C_STM_US = 0x40
    _mCH341A_CMD_I2C_STM_MS = 0x50
    _mCH341A_CMD_I2C_STM_DLY = 0x0F
    _mCH341A_CMD_I2C_STM_END = 0x00
    _mStateBitINT = 0x00000400

    IIC_SPEED_20 = 0
    IIC_SPEED_100 = 1
    IIC_SPEED_400 = 2
    IIC_SPEED_750 = 3

    def __init__(self):
        self.device_id = ctypes.c_uint32()
        self.fd = -1
        self.ic = None
        self.logger = get_logger('ch341_driver')

    def init(self):
        lib_path = ""
        try:
            package_share_directory = get_package_share_directory('tactile_sensor_ros')
            
            if os.name == 'nt':  # Windows
                lib_path = os.path.join(package_share_directory, 'lib', 'windows', 'CH341DLLA64.DLL')
            else:  # Linux
                lib_dir = os.path.join(package_share_directory, 'lib')
                possible_libs = glob.glob(os.path.join(lib_dir, '**', 'libch347.so'), recursive=True)
                if possible_libs:
                    lib_path = possible_libs[0]
                else:
                    lib_path = os.path.join(lib_dir, 'libch347.so')

            self.logger.info(f"正在加载驱动库: {lib_path}")

            if not os.path.exists(lib_path):
                self.logger.error(f"未找到库文件: {lib_path}")
                return False

            if os.name == 'nt':
                self.ic = windll.LoadLibrary(lib_path)
                self.ch341GetInput = self.ic.CH341GetInput
                self.ch341CloseDevice = self.ic.CH341CloseDevice
                self.ch341WriteData = self.ic.CH341WriteData
                self.ch341WriteRead = self.ic.CH341WriteRead
                self.ch341SetOutput = self.ic.CH341SetOutput
                self.ch341SetStream = self.ic.CH341SetStream
            else:
                self.ic = cdll.LoadLibrary(lib_path)
                self.ch341GetInput = self.ic.CH34xGetInput
                self.ch341CloseDevice = self.ic.CH34xCloseDevice
                self.ch341WriteData = self.ic.CH34xWriteData
                self.ch341WriteRead = self.ic.CH34xWriteRead
                self.ch341SetOutput = self.ic.CH34xSetOutput
                self.ch341SetStream = self.ic.CH34xSetStream

            self.logger.info("CH341 驱动库加载成功")
            return True

        except Exception as e:
            self.logger.error(f"CH341 加载失败, err = {e}")
            return False

    def open(self):
        if os.name == 'nt':
            try:
                self.fd = self.ic.CH341OpenDevice(0)
                if self.fd == -1:
                    self.logger.error("Windows: 打开设备失败")
                    return False
                self.fd = 0 
                self.logger.info("Windows: 设备已打开")
                return True
            except Exception as e:
                self.logger.error(f"Windows Open Error: {e}")
                return False
        else:
            try:
                devices = glob.glob('/dev/ch34x_pis*')
                if devices:
                    device_path = devices[0].encode()
                    self.logger.info(f"找到设备: {devices[0]}")
                    self.fd = self.ic.CH34xOpenDevice(device_path)
                    if self.fd == -1:
                        self.logger.error("Linux: 打开设备失败 (fd=-1)")
                        return False
                    else:
                        self.logger.info(f"Linux: 设备已打开 (fd={self.fd})")
                        return True
                else:
                    self.logger.error("Linux: 未找到 /dev/ch34x_pis* 设备")
                    return False
            except Exception as e:
                self.logger.error(f"Linux Open Error: {e}")
                return False

    def disconnect(self):
        if self.fd != -1 and self.ic:
            self.ch341CloseDevice(self.fd)
            self.logger.info("设备已断开")

    def connect_check(self):
        if self.ic:
            return self.ch341GetInput(self.fd, ctypes.byref(self.device_id))
        return 0

    def write(self, addr, data):
        if not self.ic: return 0
        
        s_len = len(data)
        tmp_data = list(data)
        pack = []
        cnt = 20
        pack_num = s_len // cnt
        remainder = s_len % cnt

        pack.append(self._mCH341A_CMD_I2C_STREAM)
        pack.append(self._mCH341A_CMD_I2C_STM_STA)
        pack.append(self._mCH341A_CMD_I2C_STM_OUT | 1)
        pack.append(addr << 1)

        for i in range(pack_num):
            pack.append(self._mCH341A_CMD_I2C_STM_OUT | cnt)
            pack.extend(tmp_data[0:cnt])
            del tmp_data[0:cnt]
            pack.append(self._mCH341A_CMD_I2C_STM_END)
            
            # 使用 c_ubyte 防止符号位错误
            send_buf = (c_ubyte * len(pack))(*pack)
            send_len = (c_ubyte * 1)(len(pack))
            
            if not self.ch341WriteData(self.fd, send_buf, send_len):
                return 0
            pack.clear()
            pack.append(self._mCH341A_CMD_I2C_STREAM)

        if remainder >= 1:
            pack.append(self._mCH341A_CMD_I2C_STM_OUT | remainder)
            pack.extend(tmp_data[0:remainder])

        pack.append(self._mCH341A_CMD_I2C_STM_STO)
        pack.append(self._mCH341A_CMD_I2C_STM_END)
        
        # 使用 c_ubyte
        send_buf = (c_ubyte * len(pack))(*pack)
        send_len = (c_ubyte * 1)(len(pack))
        
        if not self.ch341WriteData(self.fd, send_buf, send_len):
            return 0
        return s_len

    def read(self, addr, data):
        if not self.ic or not data: return 0
        
        r_len = len(data)
        pack = []
        read_buf = []
        
        pack_num = r_len // 30
        remainder = r_len % 30
        if remainder == 0:
            remainder = 30
            pack_num -= 1
            
        pack.append(self._mCH341A_CMD_I2C_STREAM)
        pack.append(self._mCH341A_CMD_I2C_STM_STA)
        pack.append(self._mCH341A_CMD_I2C_STM_OUT | 1)
        pack.append((addr << 1) | 0x01)
        pack.append(self._mCH341A_CMD_I2C_STM_MS | 1)
        
        for i in range(pack_num):
            pack.append(self._mCH341A_CMD_I2C_STM_IN | 30)
            pack.append(self._mCH341A_CMD_I2C_STM_END)
            
            # 使用 c_ubyte
            send_buf = (c_ubyte * len(pack))(*pack)
            rec_len = (c_ubyte * 1)()
            rec_buf = (c_ubyte * self._mCH341A_CMD_I2C_STM_MAX)()
            
            if not self.ch341WriteRead(self.fd, len(pack), send_buf, self._mCH341A_CMD_I2C_STM_MAX, 1, rec_len, rec_buf):
                return 0
            
            read_buf.extend(rec_buf[:rec_len[0]])
            pack.clear()
            pack.append(self._mCH341A_CMD_I2C_STREAM)

        if remainder > 1:
            pack.append(self._mCH341A_CMD_I2C_STM_IN | (remainder - 1))
        
        pack.append(self._mCH341A_CMD_I2C_STM_IN | 0)
        pack.append(self._mCH341A_CMD_I2C_STM_STO)
        pack.append(self._mCH341A_CMD_I2C_STM_END)
        
        # 使用 c_ubyte
        send_buf = (c_ubyte * len(pack))(*pack)
        rec_len = (c_ubyte * 1)()
        rec_buf = (c_ubyte * self._mCH341A_CMD_I2C_STM_MAX)()
        
        if not self.ch341WriteRead(self.fd, len(pack), send_buf, self._mCH341A_CMD_I2C_STM_MAX, 1, rec_len, rec_buf):
            return 0
            
        read_buf.extend(rec_buf[:rec_len[0]])
        
        data.clear()
        data.extend(read_buf)
        return len(read_buf)

    def set_speed(self, speed):
        if speed not in [self.IIC_SPEED_20, self.IIC_SPEED_100, self.IIC_SPEED_400, self.IIC_SPEED_750]:
            return False
        # 在 Linux 上 setStream 失败不一定致命，改为 Warning
        if not self.ch341SetStream(self.fd, speed):
            self.logger.warn(f"Set speed to {speed} returned False (possibly already set or driver specific behavior)")
            return False
        return True
    
    def set_int(self, lvl):
        status = (c_long * 1)()
        self.ch341GetInput(0, status)
        time.sleep(0.01)
        if lvl:
            self.ch341SetOutput(self.fd, 0x03, 0xFF00, status[0] | self._mStateBitINT)
        else:
            self.ch341SetOutput(self.fd, 0x03, 0xFF00, status[0] & (~self._mStateBitINT))