#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
压力传感器控制节点 - 独立节点（立即响应版）
订阅话题：/control/sensor_cmd
发布话题：/arm/cmd_vel (IntCmd), /control/pressure_sample_trigger (IntCmd)

功能：接收触发信号后，立即发送压力传感器开关序列（关→0.5s→开）
      并触发5秒数据采集窗口
"""

import rclpy
from rclpy.node import Node
import os
import time
import sqlite3
import json
from std_msgs.msg import Header
from robot_control_backend.msg import IntCmd
from ament_index_python.packages import get_package_share_directory

def load_env_config():
    """从 .env 文件加载配置"""
    # 修改为使用 ROS 2 标准 API 获取包的安装目录
    package_share_dir = get_package_share_directory('robot_control_backend')
    env_path = os.path.join(package_share_dir, 'rob_arm.env')
    
    if os.path.exists(env_path):
        with open(env_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    key, value = line.split('=', 1)
                    os.environ[key] = value
        print("✅ 已从 rob_arm.env 加载配置") # 此时节点还没初始化，用 print 替代 loginfo

# ====================== 数据库：强制建表，绝对成功 ======================
def setup_database(logger):
    # 直接用工作目录下的 ros_database.db，和节点A/B 完全一致
    db_path = os.environ.get('DB_PATH', "ros_database.db")
    
    # 线程安全，防止ROS报错
    conn = sqlite3.connect(db_path, check_same_thread=False)
    cursor = conn.cursor()

    # 强制建表，不存在就创建
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS sensor_logs (
        Createtime DATETIME NOT NULL,
        creater_id INTEGER NOT NULL,
        Work_ID INTEGER NOT NULL,
        sensor_ID INTEGER NOT NULL,
        isread INTEGER NOT NULL,
        data TEXT NOT NULL,
        del_flag BOOL DEFAULT false,
        Notes TEXT,
        PRIMARY KEY (Createtime, sensor_ID)
    );
    """)
    conn.commit()
    logger.info("✅ 数据库表 sensor_logs 已创建/已存在")
    return conn

# ====================== 写入数据库 ======================
def log_sensor_state(conn, sensor_id, state, action_desc, logger):
    try:
        cursor = conn.cursor()
        data_json = json.dumps({"state": state}, ensure_ascii=False)
        notes_json = json.dumps({"action": action_desc}, ensure_ascii=False)

        cursor.execute("""
        INSERT INTO sensor_logs (Createtime, creater_id, Work_ID, sensor_ID, isread, data, del_flag, Notes)
        VALUES (datetime('now'), 1, 1, ?, 2, ?, 0, ?)
        """, (sensor_id, data_json, notes_json))
        conn.commit()
        logger.info(f"✅ 数据库写入成功：sensor={sensor_id}, state={state}")
    except Exception as e:
        logger.error(f"❌ 数据库写入失败：{e}")

class PressureSensorControlNode(Node):
    def __init__(self):
        super().__init__("pressure_sensor_control_node")
        load_env_config()
        
        # 配置参数（从环境变量读取）
        self.DEV_SENSOR = int(os.environ.get('DEV_SENSOR', '49'))
        self.SAMPLE_WINDOW = float(os.environ.get('PRESSURE_SAMPLE_WINDOW', '5.0'))
        
        # 话题名称（从环境变量读取）
        TOPIC_ARM_CMD_VEL = os.environ.get('ROS_TOPIC_ARM_CMD_VEL', '/arm/cmd_vel')
        TOPIC_SENSOR_CMD = os.environ.get('ROS_TOPIC_SENSOR_CMD', '/control/sensor_cmd')
        TOPIC_SAMPLE_TRIGGER = os.environ.get('ROS_TOPIC_PRESSURE_SAMPLE_TRIGGER', '/control/pressure_sample_trigger')
        
        self.get_logger().info(f"压力传感器控制节点已启动（立即响应模式）")
        self.get_logger().info(f"📥 订阅触发话题: {TOPIC_SENSOR_CMD}")
        self.get_logger().info(f"📤 发布指令话题: {TOPIC_ARM_CMD_VEL}")
        self.get_logger().info(f"📤 发布采集触发话题: {TOPIC_SAMPLE_TRIGGER}")

        # 发布：向下位机发送压力传感器开关指令 (ROS 2 参数顺序：类型, 话题名, 队列)
        self.pub_sensor_cmd = self.create_publisher(IntCmd, TOPIC_ARM_CMD_VEL, 10)
        
        # 新增：发布采集触发信号（触发平均值计算节点）
        self.pub_sample_trigger = self.create_publisher(IntCmd, TOPIC_SAMPLE_TRIGGER, 10)

        # 订阅：触发信号
        self.create_subscription(IntCmd, TOPIC_SENSOR_CMD, self.trigger_callback, 10)

        # 数据库（自动建表）
        self.db_conn = setup_database(self.get_logger())
        
        # 用于处理 0.5s 延迟的定时器，初始为 None
        self.timer_0_5s = None
        self.target_module_id = None

    def trigger_callback(self, msg):
        """收到触发信号 → 立即发送压力传感器开关序列"""
        self.target_module_id = msg.module_id
        self.get_logger().info(f"📡 收到传感器触发指令，module_id={self.target_module_id}，立即执行")
        self.send_sensor_sequence(self.target_module_id)

    def send_sensor_sequence(self, module_id):
        """发送压力传感器开关序列（2帧，间隔0.5秒）并触发数据采集"""
        self.get_logger().info("🔌 发送压力传感器指令序列")

        # 第1帧：关闭
        cmd1 = IntCmd()
        cmd1.header = Header()
        # ROS 2 必须加 .to_msg()
        cmd1.header.stamp = self.get_clock().now().to_msg()
        cmd1.module_id = module_id
        cmd1.device_id = self.DEV_SENSOR
        cmd1.position = [0]
        self.pub_sensor_cmd.publish(cmd1)
        log_sensor_state(self.db_conn, self.DEV_SENSOR, 0, "关闭压力传感器", self.get_logger())
        self.get_logger().info(f"压力传感器指令1: module={module_id}, pos=0")

        # 延迟0.5秒发送第2帧 (使用 ROS 2 定时器非阻塞延迟，防止卡死回调)
        if self.timer_0_5s is not None:
            self.timer_0_5s.cancel() # 如果之前的定时器还在，取消它
        self.timer_0_5s = self.create_timer(0.5, self.send_second_frame_callback)

    def send_second_frame_callback(self):
        """0.5秒定时器回调，发送第2帧并触发数据采集"""
        # 只执行一次，立即销毁定时器
        if self.timer_0_5s is not None:
            self.timer_0_5s.cancel()
            self.timer_0_5s = None

        module_id = self.target_module_id

        # 第2帧：打开
        cmd2 = IntCmd()
        cmd2.header = Header()
        cmd2.header.stamp = self.get_clock().now().to_msg()
        cmd2.module_id = module_id
        cmd2.device_id = self.DEV_SENSOR
        cmd2.position = [1]
        self.pub_sensor_cmd.publish(cmd2)
        log_sensor_state(self.db_conn, self.DEV_SENSOR, 1, "打开压力传感器", self.get_logger())
        self.get_logger().info(f"压力传感器指令2: module={module_id}, pos=1")

        self.get_logger().info("✅ 压力传感器指令序列发送完成")
        
        # 发送采集触发信号，通知平均值计算节点开始采集5秒数据
        trigger_msg = IntCmd()
        trigger_msg.header = Header()
        trigger_msg.header.stamp = self.get_clock().now().to_msg()
        trigger_msg.module_id = module_id
        trigger_msg.device_id = self.DEV_SENSOR
        trigger_msg.position = [1]  # 触发信号
        self.pub_sample_trigger.publish(trigger_msg)
        self.get_logger().info(f"📡 已发送采集触发信号，开始采集 {self.SAMPLE_WINDOW} 秒数据")

    def shutdown_hook(self):
        self.db_conn.close()
        self.get_logger().info("✅ 数据库连接已关闭")

def main(args=None):
    rclpy.init(args=args)
    node = PressureSensorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
