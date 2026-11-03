#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 角度 + 末端坐标发布节点（正运动学，不漂移）
订阅：/hardware/gyroscope_feedback (GyroFeedback)
发布：/hardware/imu_angles (tuo_luo_yi)
"""

import rclpy
from rclpy.node import Node
import os
import math
import numpy as np
import sqlite3
import json
from datetime import datetime
from std_msgs.msg import Header
from robot_control_backend.msg import GyroFeedback, TuoLuoYi
from robot_control_backend.msg import _tuo_luo_yi as tuo_luo_yi
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
        print("✅ 已从 rob_arm.env 加载配置")

class ImuAnglePublisher(Node): # 继承 Node
    def __init__(self):
        super().__init__('imu_angle_publisher') # 注意：任务单要求注册名为 imu_angle_publisher
        load_env_config()
        
        # 从环境变量读取配置参数
        self.INITIAL_ROLL = float(os.environ.get('IMU_INITIAL_ROLL', '0.0'))
        self.INITIAL_PITCH = float(os.environ.get('IMU_INITIAL_PITCH', '0.0'))
        self.INITIAL_YAW = float(os.environ.get('IMU_INITIAL_YAW', '0.0'))
        
        self.GRAVITY = float(os.environ.get('IMU_GRAVITY', '9.81'))
        self.ACCEL_UNIT_IS_G = os.environ.get('IMU_ACCEL_UNIT_IS_G', 'False').lower() == 'true'
        
        self.KP = float(os.environ.get('IMU_KP', '0.5'))
        self.KI = float(os.environ.get('IMU_KI', '0.05'))
        self.ACC_TRUST_THRESH = float(os.environ.get('IMU_ACC_TRUST_THRESH', '1.0'))
        
        self.ARM_BASE_IDS = [int(x) for x in os.environ.get('IMU_ARM_BASE_IDS', '32,64,96').split(',')]
        self.GYRO_OFFSET = int(os.environ.get('IMU_GYRO_OFFSET', '18'))
        
        self.BASE_X = float(os.environ.get('IMU_BASE_X', '0.0'))
        self.BASE_Y = float(os.environ.get('IMU_BASE_Y', '0.0'))
        self.BASE_Z = float(os.environ.get('IMU_BASE_Z', '0.0'))
        self.SWING_CENTER_HEIGHT = float(os.environ.get('IMU_SWING_CENTER_HEIGHT', '30.0'))
        self.TELESCOPIC_LENGTH = float(os.environ.get('IMU_TELESCOPIC_LENGTH', '50.0'))
        
        # 从环境变量读取话题名称
        self.TOPIC_GYROSCOPE_FEEDBACK = os.environ.get('ROS_TOPIC_GYROSCOPE_FEEDBACK', '/hardware/gyroscope_feedback')
        self.TOPIC_IMU_ANGLES = os.environ.get('ROS_TOPIC_IMU_ANGLES', '/hardware/imu_angles')
        
        self.states = {}       # key: (module_id, arm_id)

        # 订阅 (ROS 2 参数顺序：消息类型, 话题名, 回调, 队列)
        self.create_subscription(GyroFeedback, self.TOPIC_GYROSCOPE_FEEDBACK, self.imu_callback, 10)
        # 发布
        self.angle_pub = self.create_publisher(TuoLuoYi, self.TOPIC_IMU_ANGLES, 10)

        # ------------------ 数据库初始化 ------------------
        db_path = os.environ.get('DB_PATH', "ros_database.db")
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self._create_table()
        self.get_logger().info("✅ IMU节点数据库已连接")

        self.get_logger().info("IMU 角度+坐标节点启动 (正运动学)")

    def _create_table(self):
        """创建 sensor_records 表（如果不存在）"""
        self.conn.execute("""
            CREATE TABLE IF NOT EXISTS sensor_records (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                sensor_ID INTEGER NOT NULL,
                sensordescript TEXT,
                IsRead INTEGER NOT NULL,
                Module_ID INTEGER NOT NULL,
                Unit_ID INTEGER NOT NULL,
                Unit_address INTEGER NOT NULL,
                unit_row_id INTEGER NOT NULL,
                creater_id INTEGER NOT NULL,
                Createtime DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
                del_flag BOOL DEFAULT false,
                Notes TEXT
            )
        """)
        self.conn.commit()

    def _save_record(self, module_id, device_id, arm_id, swing, rotation, x, y, z):
        """保存IMU计算结果到数据库"""
        unit = arm_id * 32  # 臂1→32, 臂2→64, 臂3→96
        sensor_ID = 50  # 按照要求将 device_id 改为 50
        sensordescript = f"IMU传感器-臂{arm_id}"
        IsRead = 1  # 1=读，IMU为只读传感器
        Unit_address = device_id  # 单元中定义的地址
        unit_row_id = arm_id  # 所属机械臂数据库主键

        # 构建备注信息，包含角度和坐标数据
        data_dict = {
            "swing_angle": round(swing, 2),
            "rotation_angle": round(rotation, 2),
            "x": round(x, 2),
            "y": round(y, 2),
            "z": round(z, 2)
        }
        notes = json.dumps(data_dict, ensure_ascii=False)

        try:
            self.conn.execute("""
                INSERT INTO sensor_records
                (sensor_ID, sensordescript, IsRead, Module_ID, Unit_ID, Unit_address, unit_row_id, creater_id, del_flag, Notes)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                sensor_ID,         # sensor_ID: 传感器/电机硬件协议编号
                sensordescript,    # sensordescript: 传感器描述
                IsRead,            # IsRead: 读写方式
                module_id,         # Module_ID: 所属模块ID
                unit,              # Unit_ID: 所属机械臂硬件/协议编号
                Unit_address,      # Unit_address: 单元中定义的地址
                unit_row_id,       # unit_row_id: 所属机械臂数据库主键
                1,                 # creater_id: 创建者ID
                False,             # del_flag: 删除标志
                notes              # Notes: 备注信息（包含角度和坐标）
            ))
            self.conn.commit()
        except Exception as e:
            self.get_logger().error(f"数据库写入失败: {e}")

    def shutdown_hook(self):
        self.conn.close()
        self.get_logger().info("✅ IMU节点数据库连接已关闭")

    def _get_arm_id(self, device_id):
        base = device_id - self.GYRO_OFFSET
        if base in self.ARM_BASE_IDS:
            return self.ARM_BASE_IDS.index(base) + 1
        return None

    def _ensure_state(self, module_id, arm_id):
        key = (module_id, arm_id)
        if key not in self.states:
            self.states[key] = {
                'quat':     self._euler_to_quat(self.INITIAL_ROLL, self.INITIAL_PITCH, self.INITIAL_YAW),
                'eInt':     np.zeros(3),
                'last_time': None
            }
        return self.states[key]

    @staticmethod
    def _euler_to_quat(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return np.array([w, x, y, z])

    @staticmethod
    def _quat_multiply(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        ])

    @staticmethod
    def _quat_normalize(q):
        norm = np.linalg.norm(q)
        if norm < 1e-10:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return q / norm

    def _quat_to_euler_zyx_deg(self, quat):
        w, x, y, z = quat
        sin_pitch = 2.0 * (w * y - z * x)
        sin_pitch = max(-1.0, min(1.0, sin_pitch))
        pitch = math.asin(sin_pitch)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def imu_callback(self, msg):
        module_id = msg.module_id
        device_id = msg.device_id
        arm_id = self._get_arm_id(device_id)
        if arm_id is None:
            self.get_logger().warn(f"未识别的 device_id: {device_id}，忽略")
            return

        state = self._ensure_state(module_id, arm_id)

        # ROS 2 获取当前时间的正确方式
        now = self.get_clock().now().nanoseconds / 1e9
        if state['last_time'] is None:
            state['last_time'] = now
            return
        dt = now - state['last_time']
        state['last_time'] = now
        if dt <= 0:
            return

        # 数据预处理
        accel = np.array([msg.accel_x, msg.accel_y, msg.accel_z])
        if self.ACCEL_UNIT_IS_G:
            accel *= self.GRAVITY
        gyro = np.radians([msg.gyro_x, msg.gyro_y, msg.gyro_z])

        # Mahony 互补滤波
        gyro_corrected = gyro.copy()
        acc_norm = np.linalg.norm(accel)
        if abs(acc_norm - self.GRAVITY) < self.ACC_TRUST_THRESH and acc_norm > 1e-6:
            acc_unit = accel / acc_norm
            w, x, y, z = state['quat']
            R = np.array([
                [1 - 2*y*y - 2*z*z,   2*x*y - 2*w*z,       2*x*z + 2*w*y],
                [2*x*y + 2*w*z,       1 - 2*x*x - 2*z*z,   2*y*z - 2*w*x],
                [2*x*z - 2*w*y,       2*y*z + 2*w*x,       1 - 2*x*x - 2*y*y]
            ])
            gravity_est = R[2, :]   # 世界系 Z 轴在机体系的投影
            error = np.cross(acc_unit, gravity_est)
            state['eInt'] += error * self.KI * dt
            correction = self.KP * error + state['eInt']
            gyro_corrected = gyro + correction

        # 四元数更新（右乘）
        omega_norm = np.linalg.norm(gyro_corrected)
        if omega_norm > 1e-6:
            axis = gyro_corrected / omega_norm
            angle = omega_norm * dt
            dq = np.array([math.cos(angle/2),
                           axis[0] * math.sin(angle/2),
                           axis[1] * math.sin(angle/2),
                           axis[2] * math.sin(angle/2)])
            state['quat'] = self._quat_multiply(state['quat'], dq)
            state['quat'] = self._quat_normalize(state['quat'])

        # 提取欧拉角并符号适配
        _, pitch_deg, yaw_deg = self._quat_to_euler_zyx_deg(state['quat'])
        swing = pitch_deg                     # 前倾为正
        rotation = -yaw_deg                   # 顺时针为正

        # ===== 正运动学坐标计算 =====
        swing_rad = math.radians(swing)
        rot_rad = math.radians(rotation)

        # 末端相对底座的位置
        dx_rel = self.TELESCOPIC_LENGTH * math.sin(swing_rad) * math.cos(rot_rad)
        dy_rel = self.TELESCOPIC_LENGTH * math.sin(swing_rad) * math.sin(rot_rad)
        dz_rel = self.SWING_CENTER_HEIGHT + self.TELESCOPIC_LENGTH * math.cos(swing_rad)

        # 世界坐标
        x = self.BASE_X + dx_rel
        y = self.BASE_Y + dy_rel
        z = self.BASE_Z + dz_rel

        # 发布消息
        angle_msg = TuoLuoYi()
        # ROS 2 时间戳必须转 .to_msg()
        angle_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="world")
        angle_msg.module_id = module_id
        angle_msg.device_id = device_id
        angle_msg.swing_angle = swing
        angle_msg.rotation_angle = rotation
        angle_msg.x = x
        angle_msg.y = y
        angle_msg.z = z
        self.angle_pub.publish(angle_msg)

        # 写入数据库
        self._save_record(module_id, device_id, arm_id, swing, rotation, x, y, z)

def main(args=None):
    rclpy.init(args=args)
    node = ImuAnglePublisher()
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
