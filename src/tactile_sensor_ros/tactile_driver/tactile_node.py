#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# 导入自定义消息
# 注意：在混合编译模式下，消息包名通常就是包名
from tactile_sensor_ros.msg import FingerData, TactileArray


# 导入底层驱动 
from tactile_driver.ch341_driver import Ch341Driver
from tactile_driver.finger import Finger
from tactile_driver.sensor_para import finger_params

class TactileSensorNode(Node):
    # CH341 连接状态枚举
    CH341_CONNECT_INIT = 0
    CH341_CONNECT_OPEN = 1
    CH341_CONNECT_SET_SPEED = 2
    CH341_CONNECT_SAMPLE_START = 3
    CH341_CONNECT_CHECK = 4
    CH341_CONNECT_SAMPLE_STOP = 5

    def __init__(self):
        super().__init__('tactile_sensor_node')
        
        # 1. 声明参数
        self.declare_parameter('polling_rate', 100.0)  # 默认 100Hz
        self.declare_parameter('max_finger_num', 5)
        self.declare_parameter('pca_base_addr', 0x70)

        self.polling_rate = self.get_parameter('polling_rate').value
        self.max_finger_num = self.get_parameter('max_finger_num').value
        self.pca_addr = self.get_parameter('pca_base_addr').value

        # 2. 初始化发布者
        self.pub_tactile = self.create_publisher(TactileArray, 'tactile_data', 10)

        # 3. 初始化驱动对象
        self.ch341 = Ch341Driver()
        self.fingers = []
        # 初始化手指对象 (IIC 地址从 2 开始递增)
        for i in range(self.max_finger_num):
            self.fingers.append(Finger(2 + i, self.ch341))

        # 4. 内部状态变量
        self.connect_status = self.CH341_CONNECT_INIT
        self.ch341_check_timer = 0
        self.sync_timer = 0
        self.last_sync_time = 0
        
        # 5. 启动主循环定时器
        timer_period = 1.0 / self.polling_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Tactile Sensor Node Started')

    def timer_callback(self):
        """
        主循环：处理连接状态机和数据读取
        """
        if self.connect_status == self.CH341_CONNECT_INIT:
            if self.ch341.init():
                self.connect_status = self.CH341_CONNECT_OPEN
            else:
                # 如果加载库失败，稍微等待避免刷屏日志，虽然 init 内部已有日志
                time.sleep(1.0)

        elif self.connect_status == self.CH341_CONNECT_OPEN:
            if self.ch341.open():
                self.connect_status = self.CH341_CONNECT_SET_SPEED
            else:
                self.connect_status = self.CH341_CONNECT_INIT
                time.sleep(1.0) # 重试等待

        elif self.connect_status == self.CH341_CONNECT_SET_SPEED:
            # 尝试设置速度 400k
            if self.ch341.set_speed(self.ch341.IIC_SPEED_400):
                self.ch341.set_int(0)
                time.sleep(0.5) # 稍微减少 sleep，避免阻塞太久
                self.ch341.set_int(1)
                self.connect_status = self.CH341_CONNECT_SAMPLE_START
            else:
                self.get_logger().error("Set IIC speed failed")
                self.connect_status = self.CH341_CONNECT_SAMPLE_START

        elif self.connect_status == self.CH341_CONNECT_SAMPLE_START:
            self.connect_status = self.CH341_CONNECT_CHECK
            self.get_logger().info("Start Sampling...")

        elif self.connect_status == self.CH341_CONNECT_CHECK:
            # 周期性检查 CH341 连接状况 (每 1秒 check 一次)
            self.ch341_check_timer += 1
            if self.ch341_check_timer >= self.polling_rate: # 约1秒
                self.ch341_check_timer = 0
                if self.ch341.connect_check() == 0:
                    self.get_logger().warn("CH341 Disconnected!")
                    self.connect_status = self.CH341_CONNECT_SAMPLE_STOP
            
            # --- 核心读取逻辑 ---
            self.read_and_publish()

        elif self.connect_status == self.CH341_CONNECT_SAMPLE_STOP:
            self.sync_timer = 0
            for f in self.fingers:
                f.disconnected()
            self.ch341.disconnect()
            self.connect_status = self.CH341_CONNECT_INIT

    def read_and_publish(self):
        """
        读取所有传感器数据并发布
        """
        tactile_msg = TactileArray()
        tactile_msg.header.stamp = self.get_clock().now().to_msg()
        tactile_msg.header.frame_id = "tactile_sensor_frame"

        connected_sensor_chan = 0
        connected_sensor_cnt = 0

        for idx, finger in enumerate(self.fingers):
            # 激活对应通道 (简单的位操作模拟多路复用)
            # set_sensor_enable 逻辑：写入 PCA 芯片控制通道
            # 原代码逻辑：self.ch341.write(self.pcaAddr, [1 << finger.pcaIdx])
            # 注意：finger.pcaIdx 是 2+i，需要根据实际硬件确认
            try:
                # 假设 pca_addr 是 0x70，发送 1字节通道掩码
                self.ch341.write(self.pca_addr, [1 << finger.pca_idx])
                connected_sensor_chan |= (1 << finger.pca_idx)
            except Exception as e:
                self.get_logger().debug(f"IIC Mux Write Failed: {e}")

            if not finger.connect:
                # 尝试连接
                if finger.check_sensor():
                    self.get_logger().info(f"Sensor [{idx}] Connected at addr {finger.addr}")
            else:
                # 读取数据
                if finger.cap_read():
                    # 数据转换: Internal Class -> ROS Msg
                    finger_data_msg = FingerData()
                    finger_data_msg.sensor_index = idx
                    
                    # 填充 List 数据
                    finger_data_msg.channel_cap_data = [int(x) for x in finger.read_data.channel_cap_data]
                    finger_data_msg.nf = [float(x) for x in finger.read_data.nf]
                    finger_data_msg.tf = [float(x) for x in finger.read_data.tf]
                    finger_data_msg.tf_dir = [int(x) for x in finger.read_data.tf_dir]
                    finger_data_msg.s_prox_cap_data = [int(x) for x in finger.read_data.s_prox_cap_data]
                    finger_data_msg.m_prox_cap_data = [int(x) for x in finger.read_data.m_prox_cap_data]

                    tactile_msg.fingers.append(finger_data_msg)
                    connected_sensor_cnt += 1

        # 发布消息 (即使没有手指连接，也可以发布空消息心跳)
        self.pub_tactile.publish(tactile_msg)

        # 同步逻辑 (1秒一次)
        now = time.time()
        if connected_sensor_cnt > 1 and (now - self.last_sync_time) > 1.0:
            self.last_sync_time = now
            # 开启所有通道
            self.ch341.write(self.pca_addr, [connected_sensor_chan])
            self.ch341.set_int(1)
            # 向第一个连接的手指发送同步命令
            for f in self.fingers:
                if f.connect:
                    f.sns_cmd.set_sensor_sync(0)
                    break

def main(args=None):
    rclpy.init(args=args)
    node = TactileSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        if node.ch341:
            node.ch341.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()