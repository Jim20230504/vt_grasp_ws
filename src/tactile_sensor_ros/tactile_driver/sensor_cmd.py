import time
from ctypes import c_uint8
from rclpy.logging import get_logger

class SensorCmd:
    def __init__(self, ch341):
        self.logger = get_logger('sensor_cmd')
        self.CMD_GET_CHANNEL_NUM = 0x01
        self.CMD_GET_SENSOR_CAP_DATA = 0x60
        self.CMD_GET_SENSOR_CHANNEL = 0x62
        self.CMD_SET_SENSOR_AUTO_DAC = 0x63
        self.CMD_GET_SENSOR_ERR_CODE = 0x64
        self.CMD_GET_SENSOR_TEST_HZ = 0x6C
        self.CMD_SET_SENSOR_IIC_ADDR = 0x70
        self.CMD_GET_SENSOR_IIC_ADDR = 0x71
        self.CMD_SET_SENSOR_CDC_SYNC = 0x72
        self.CMD_SET_SENSOR_CDC_START_OFFSET = 0x73
        self.CMD_SET_SENSOR_RESTART = 0x77
        self.CMD_SET_SENSOR_SEND_TYPE = 0x7F
        self.CMD_GET_VERSION = 0xA0
        self.CMD_SOFT_RESTART = 0xA1
        self.CMD_GET_TYPE = 0xA2
        self.CMD_SET_TYPE = 0xA3
        self.CMD_SET_INF = 0xA5
        self.CMD_GET_PRG = 0xA6

        self._ch341 = ch341

    def calc_sum(self, pack):
        if len(pack) <= 5: return 0
        _sum = sum(pack[i] & 0xFF for i in range(len(pack)))
        pack.append(_sum & 0xFF)
        pack.append((_sum >> 8) & 0xFF)

    def check_sum(self, pack):
        if len(pack) <= 5: return False
        _sum = sum(pack[i] & 0xFF for i in range(len(pack) - 2))
        chk_l = _sum & 0xFF
        chk_h = (_sum >> 8) & 0xFF
        if chk_l == (pack[-2] & 0xFF) and chk_h == (pack[-1] & 0xFF):
            return True
        return False

    def get_addr(self, addr):
        _pack = [0xAA, 0x55, 0x03, self.CMD_GET_SENSOR_IIC_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.calc_sum(_pack)
        self._ch341.write(addr, _pack)
        
        _pack = [0] * 11
        time.sleep(0.01)
        self._ch341.read(addr, _pack)
        
        if self.check_sum(_pack):
            return _pack[7] & 0xFF
        return 0

    def set_sensor_send_type(self, addr, send_type):
        _pack = [0xAA, 0x55, 0x03, self.CMD_SET_SENSOR_SEND_TYPE, 0x00, 0x00, 0x00, send_type, 0x00]
        self.calc_sum(_pack)
        self._ch341.write(addr, _pack)
        
        _pack = [0] * 11
        time.sleep(0.01)
        self._ch341.read(addr, _pack)
        return self.check_sum(_pack) and ((self.CMD_SET_SENSOR_SEND_TYPE | 0x80) == c_uint8(_pack[3]).value)

    def set_sensor_cap_offset(self, addr, offset):
        _pack = [0xAA, 0x55, 0x03, self.CMD_SET_SENSOR_CDC_START_OFFSET, 0x00, 0x00, 0x00, offset, 0x00]
        self.calc_sum(_pack)
        self._ch341.write(addr, _pack)
        
        _pack = [0] * 6
        time.sleep(0.01)
        self._ch341.read(addr, _pack)
        return self.check_sum(_pack) and ((self.CMD_SET_SENSOR_CDC_START_OFFSET | 0x80) == c_uint8(_pack[3]).value)

    def get_sensor_project_index(self, addr):
        _pack = [0xAA, 0x55, 0x03, self.CMD_GET_PRG, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.calc_sum(_pack)
        self._ch341.write(addr, _pack)
        
        _pack = [0] * 11
        time.sleep(0.01)
        self._ch341.read(addr, _pack)
        if self.check_sum(_pack):
            return (_pack[7] & 0xFF) + (_pack[8] & 0xFF) * 256
        return 0

    def get_sensor_cap_data(self, addr, buf):
        tar_len = len(buf)
        err = self._ch341.read(addr, buf)
        # if err == 0: self.logger.error("Read Data Error")
        
        if len(buf) != tar_len:
            buf.clear()
            buf.extend([0]*tar_len)
            return False

        if (buf[0] & 0xFF) == 0x55 and (buf[1] & 0xFF) == 0xAA and self.check_sum(buf):
            return True
        return False

    def set_sensor_sync(self, addr):
        _pack = [0xAA, 0x55, 0x03, self.CMD_SET_SENSOR_CDC_SYNC, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.calc_sum(_pack)
        self._ch341.write(addr, _pack)
        return True
    
    def set_addr(self, addr, new_addr):
        # 简化实现
        return 0