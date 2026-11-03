#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
压力传感器平均值计算节点
- 订阅压力传感器原始数据中转话题
- 订阅采集触发话题，收到触发后采集5秒数据计算平均值
- 发布平均值到传感器反馈话题（供前端使用）
"""
import rclpy
from rclpy.node import Node
import os
import numpy as np
from std_msgs.msg import Header
from robot_control_backend.msg import SensorCmd, IntCmd
from ament_index_python.packages import get_package_share_directory

def load_env_config():
    """从 .env 文件加载配置"""
    # 使用 ROS 2 标准 API 获取包的安装目录
    package_share_dir = get_package_share_directory('robot_control_backend')
    env_path = os.path.join(package_share_dir, 'rob_arm.env')
    if os.path.exists(env_path):
        with open(env_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    key, value = line.split('=', 1)
                    os.environ[key] = value
        print("✅ 已从 rob_arm.env 加载配置") # 节点未初始化前用 print

class PressureAverageCalculator(Node): # 继承 Node
    def __init__(self):
        """初始化平均值计算器"""
        super().__init__('pressure_average_calculator') # 调用父类初始化
        load_env_config()
        
        # 从环境变量读取配置参数
        self.SAMPLE_WINDOW = float(os.environ.get('PRESSURE_SAMPLE_WINDOW', '5.0'))
        self.WINDOW_SIZE = int(os.environ.get('PRESSURE_WINDOW_SIZE', '50'))
        # 🚨 修复历史问题：移除硬编码的 MODULE_ID=17，改为动态接收
        self.DEV_SENSOR = int(os.environ.get('DEV_SENSOR', '49'))
        
        # 从环境变量读取话题名称
        TOPIC_SENSOR_RAW = os.environ.get('ROS_TOPIC_SENSOR_RAW', '/hardware/sensor_raw')
        TOPIC_SENSOR_AVG = os.environ.get('ROS_TOPIC_SENSOR_AVG', '/hardware/sensor_feedback')
        TOPIC_SAMPLE_TRIGGER = os.environ.get('ROS_TOPIC_PRESSURE_SAMPLE_TRIGGER', '/control/pressure_sample_trigger')
        
        # 数据缓存
        self.data_buffer = []  # 存储原始数据
        self.is_collecting = False  # 是否正在采集数据
        self.collect_start_time = None  # 采集开始时间
        self.current_module_id = None  # 🚨 新增：记录当前正在采集的模块ID
        
        # 订阅压力传感器原始数据话题 (ROS 2 参数顺序：消息类型, 话题名, 回调, 队列)
        self.sensor_sub = self.create_subscription(
            SensorCmd,
            TOPIC_SENSOR_RAW,
            self.sensor_callback,
            10
        )
        
        # 订阅采集触发话题
        self.trigger_sub = self.create_subscription(
            IntCmd,
            TOPIC_SAMPLE_TRIGGER,
            self.trigger_callback,
            10
        )
        
        # 发布平均值结果到传感器反馈话题
        self.avg_pub = self.create_publisher(
            SensorCmd,
            TOPIC_SENSOR_AVG,
            10
        )
        
        self.get_logger().info("🧮 压力传感器平均值计算节点已启动")
        self.get_logger().info(f"📥 订阅原始数据话题: {TOPIC_SENSOR_RAW}")
        self.get_logger().info(f"📥 订阅触发话题: {TOPIC_SAMPLE_TRIGGER}")
        self.get_logger().info(f"📤 发布平均值话题: {TOPIC_SENSOR_AVG}")
        self.get_logger().info(f"⏱️  采集窗口: {self.SAMPLE_WINDOW}秒 ({self.WINDOW_SIZE}个数据点)")
    
    def trigger_callback(self, msg):
        """
        采集触发回调函数
        收到触发信号后，开始采集数据窗口
        """
        self.get_logger().info(f"📡 收到采集触发信号，module_id={msg.module_id}")
        
        # 清空缓存，开始新的采集窗口
        self.data_buffer = []
        self.is_collecting = True
        # 🚨 修复：记录动态传入的 module_id，而不是死用 17
        self.current_module_id = msg.module_id 
        
        # ROS 2 获取当前时间的正确方式
        self.collect_start_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info(f"⏱️  开始采集 {self.SAMPLE_WINDOW} 秒的压力传感器数据")
    
    def sensor_callback(self, msg):
        """
        传感器数据回调函数
        接收原始数据并缓存
        """
        # 如果正在采集数据，将数据加入缓存
        if self.is_collecting:
            # ROS 2 获取当前时间的正确方式
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # 检查是否超过采集窗口时间
            if current_time - self.collect_start_time >= self.SAMPLE_WINDOW:
                # 采集窗口结束，计算平均值并发布
                self.calculate_and_publish_average()
                self.is_collecting = False
                return
            
            # 将数据加入缓存
            if len(msg.position) > 0:
                self.data_buffer.append(msg.position[0])
                
                # 如果缓存超过窗口大小，移除最旧的数据
                if len(self.data_buffer) > self.WINDOW_SIZE:
                    self.data_buffer.pop(0)
                
                self.get_logger().info(f"📊 采集数据 #{len(self.data_buffer)}: {msg.position[0]:.2f} N")
    
    def calculate_and_publish_average(self):
        """
        计算平均值并发布结果
        """
        if not self.data_buffer:
            self.get_logger().warn("⚠️  没有采集到数据，无法计算平均值")
            return
        
        # 计算平均值
        average_value = np.mean(self.data_buffer)
        
        # 构建平均值消息
        avg_msg = SensorCmd()
        avg_msg.header = Header()
        # 时间戳必须转 .to_msg()
        avg_msg.header.stamp = self.get_clock().now().to_msg()
        avg_msg.header.frame_id = "pressure_avg"
        avg_msg.id = 0
        # 🚨 修复：使用动态记录的 module_id
        avg_msg.module_id = self.current_module_id 
        avg_msg.device_id = self.DEV_SENSOR
        avg_msg.position = [float(average_value)]
        
        # 发布平均值结果
        self.avg_pub.publish(avg_msg)
        
        self.get_logger().info(f"✅ 已发布压力传感器平均值: {average_value:.2f} N (采集了 {len(self.data_buffer)} 个数据点, module_id={self.current_module_id})")
        
        # 清空缓存
        self.data_buffer = []

def main(args=None):
    rclpy.init(args=args)
    calculator = PressureAverageCalculator()
    try:
        rclpy.spin(calculator)
    except KeyboardInterrupt:
        pass
    finally:
        calculator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
