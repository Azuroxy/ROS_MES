#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
控制节点：第一套逆解时序转发，第二套前端微调转发，统一管理压力传感器。
所有话题强制从 .env 配置文件读取，无默认值。
"""
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node  # 补充：必须导入 Node 类
from rclpy.executors import MultiThreadedExecutor  # 补充：多线程执行器
import os
import time  # 补充：time.sleep 需要导入 time
import threading
from collections import defaultdict
from std_msgs.msg import Header
from robot_control_backend.msg import RotationCmd, SwingCmd, TelescopicCmd, IntCmd


def load_env_config():
    """从 .env 文件加载所有配置到环境变量（文件必须存在）"""
    # 使用 ROS 2 标准 API 获取包的安装目录
    package_share_dir = get_package_share_directory('robot_control_backend')
    env_path = os.path.join(package_share_dir, 'rob_arm.env')
    
    if not os.path.exists(env_path):
        raise FileNotFoundError(f"配置文件 {env_path} 未找到")
    with open(env_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                key, value = line.split('=', 1)
                os.environ[key] = value
    print("✅ 控制节点已从 rob_arm.env 加载配置")


class ArmSequenceController(Node):
    """控制节点：时序转发逆解指令和前端微调指令，统一管理压力传感器触发"""
    def __init__(self):
        super().__init__("arm_sequence_controller")
        load_env_config()   # 先加载配置
        #输出话题（ROS 2的参数顺序:消息类型 话题名 队列长度)
        # ---------- 输出话题（发布给功能节点，强制从 .env 读取）----------
        # 修复：create_Publisher 改为 create_publisher，且 ROS 2 不需要 queue_size= 关键字，直接填 10 即可
        self.pub_rot_seq = self.create_publisher(
            RotationCmd, os.environ['ROS_TOPIC_KINEMATICS_ROTATION_CMD_SEQ'],       # 如 /control/kinematics_rotation_cmd_sequenced
            10)
        self.pub_sw_seq = self.create_publisher(
            SwingCmd, os.environ['ROS_TOPIC_KINEMATICS_SWING_CMD_SEQ'],
            10)
        self.pub_tel_seq = self.create_publisher(
            TelescopicCmd, os.environ['ROS_TOPIC_KINEMATICS_TELESCOPIC_CMD_SEQ'],
            10)

        # 压力传感器触发话题（发布给独立压力传感器节点）
        self.pub_sensor_trigger = self.create_publisher(
            IntCmd, os.environ['ROS_TOPIC_SENSOR_CMD'],                    # 如 /control/sensor_cmd
            10)

        # ---------- 第一套功能：逆解指令缓冲 ----------
        self.buffer_lock = threading.Lock()
        # 缓冲结构：module_id -> arm_idx -> joint_name -> 对应的消息
        self.cmd_buffer = defaultdict(lambda: defaultdict(dict))
        self.ready_queue = []                                      # 已集齐的模块指令队列
        self.seq_thread = threading.Thread(target=self._sequence_worker)# 启动独立线程执行时序（不阻塞 ROS 主线程）
        self.seq_thread.daemon = True
        self.seq_thread.start()

        # ---------- 压力传感器去抖定时器 ----------
        self.timers_lock = threading.Lock()
        self.adjust_timers = {}                                    # 每个模块一个7秒定时器

        # ---------- 订阅逆运算原始话题（强制从 .env 读取）----------
        self.create_subscription(
            RotationCmd, os.environ['ROS_TOPIC_KINEMATICS_ROTATION_CMD_INPUT'],
            self.cb_rotation, qos_profile_sensor_data)
        self.create_subscription(
            SwingCmd, os.environ['ROS_TOPIC_KINEMATICS_SWING_CMD_INPUT'],
            self.cb_swing, qos_profile_sensor_data)
        self.create_subscription(
            TelescopicCmd, os.environ['ROS_TOPIC_KINEMATICS_TELESCOPIC_CMD_INPUT'],
            self.cb_telescopic, qos_profile_sensor_data)

        # ---------- 第二套功能：订阅前端微调话题（使用传感器 QoS）----------
        self.create_subscription(
            RotationCmd, os.environ['ROS_TOPIC_ADJUST_ROTATION_CMD_INPUT'],
            self.cb_adjust_rotation, qos_profile_sensor_data)
        self.create_subscription(
            SwingCmd, os.environ['ROS_TOPIC_ADJUST_SWING_CMD_INPUT'],
            self.cb_adjust_swing, qos_profile_sensor_data)
        self.create_subscription(
            TelescopicCmd, os.environ['ROS_TOPIC_ADJUST_TELESCOPIC_CMD_INPUT'],
            self.cb_adjust_telescopic, qos_profile_sensor_data)

        self.get_logger().info("🔧 控制节点启动（所有话题强制从 .env 读取）")

    # ---------- device_id 解析 ----------
    def _parse_device(self, device_id):
        """根据新规则 (33/34/35, 65/66/67, 97/98/99) 返回 (臂索引, 关节名)"""
        base = 32
        arm_idx = (device_id - 1) // base - 1           # 33→0, 65→1, 97→2
        joint_code = (device_id - 1) % base + 1         # 1=旋转, 2=摆动, 3=伸缩
        joint_map = {1: "rotate", 2: "swing", 3: "extend"}
        return arm_idx, joint_map[joint_code]

    # ---------- 压力传感器触发管理 ----------
    def _send_pressure_trigger(self, module_id):
        """向 /control/sensor_cmd 发送触发消息，由独立节点执行硬件开关"""
        trigger = IntCmd()
        trigger.header = Header(stamp=self.get_clock().now().to_msg())
        trigger.module_id = module_id
        trigger.device_id = 0
        trigger.position = [0]
        self.pub_sensor_trigger.publish(trigger)
        self.get_logger().info(f"📡 已发送压力传感器触发消息 (module={module_id})")

    def _schedule_pressure(self, module_id):
        """为指定模块启动/重置一个7秒的去抖定时器"""
        with self.timers_lock:
            if module_id in self.adjust_timers:
                self.adjust_timers[module_id].cancel()           # 取消旧定时器

            def timer_callback():
                self._on_pressure_timeout(module_id)
                # 如果只要触发一次 可在回调里销毁自己（但rclpy 没直接销毁单个定时器的 API ,通过状态位控制）

            # rclpy 的 Timer 默认是循环，为模拟单次触发，在回调里做个标记
            new_timer = self.create_timer(7.0, lambda: self._on_pressure_timeout(module_id))
            self.adjust_timers[module_id] = new_timer
            self.get_logger().info(f"⏳ 模块{module_id} 压力传感器定时器已启动/重置 (7s)")

    def _on_pressure_timeout(self, module_id):
        """定时器到期，真正发送压力传感器触发指令"""
        with self.timers_lock:
            if module_id in self.adjust_timers:
                self.adjust_timers[module_id].cancel()
                del self.adjust_timers[module_id]
        self._send_pressure_trigger(module_id)

    # ================= 第一套功能：逆解指令收集与顺序执行 =================
    def cb_rotation(self, msg): self._store_cmd(msg, "rotate")
    def cb_swing(self, msg): self._store_cmd(msg, "swing")
    def cb_telescopic(self, msg):self._store_cmd(msg, "extend")

    def _store_cmd(self, msg, joint_name):
        """将一条指令存入缓冲，当模块的3臂×3关节都集齐后打包放入执行队列"""
        arm_idx, _ = self._parse_device(msg.device_id)
        mid = msg.module_id
        with self.buffer_lock:
            self.cmd_buffer[mid][arm_idx][joint_name] = msg
            if len(self.cmd_buffer[mid]) >= 3 and all(
                len(self.cmd_buffer[mid].get(i, {})) == 3 for i in range(3)
            ):
                pkg = {
                    "module_id": mid,
                    "arms": {i: self.cmd_buffer[mid][i].copy() for i in sorted(self.cmd_buffer[mid].keys())}
                }
                self.ready_queue.append(pkg)
                del self.cmd_buffer[mid]
                self.get_logger().info(f"✅ 模块{mid} 指令集齐，加入队列")
    #时序执行线程
    def _sequence_worker(self):
        """顺序执行线程，不断从队列取出模块指令，按阶段发送"""
        while rclpy.ok():
            if self.ready_queue:
                with self.buffer_lock:
                    task = self.ready_queue.pop(0) if self.ready_queue else None
                if task:
                    self._execute_module(task)
            else:
                time.sleep(0.1) # 在子线程里用原生 time.sleep

    def _execute_module(self, task):
        """执行一个完整模块的逆解指令：旋转→8s→摆动→8s→伸缩→7s后触发压力传感器"""
        mid = task["module_id"]
        arms = task["arms"]
        try:
            # 旋转阶段：同时向三臂发布旋转指令
            for arm_idx in sorted(arms.keys()):
                msg = arms[arm_idx]["rotate"]
                msg.header.stamp = self.get_clock().now().to_msg() # 修复：node.get_clock() 改为 self.get_clock()
                #识别空指令（position=0）时直接跳下一个命令
                if hasattr(msg, 'position') and len(msg.position) > 0 and msg.position[0] == 0:
                   self.get_logger().info(f"[逆运算] ⏭️ 模块{mid} Arm{arm_idx+1} 空旋转指令，跳过")
                   continue

                # 修复致命缩进：发布动作必须在 if 判断外面，不能放在 continue 下面
                self.pub_rot_seq.publish(msg)
                self.get_logger().info(f"[逆运算] 🚀 模块{mid} Arm{arm_idx + 1} 旋转增量 {msg.position[0]:.2f}°")
            
            time.sleep(8.0)  # 使用 Python 原生休眠，不阻塞 ROS 主线程

            # 摆动阶段
            for arm_idx in sorted(arms.keys()):
                msg = arms[arm_idx]["swing"]
                msg.header.stamp = self.get_clock().now().to_msg()
                if hasattr(msg, 'position') and len(msg.position) > 0 and msg.position[0] == 0:
                    continue
                self.pub_sw_seq.publish(msg)
                self.get_logger().info(f"[逆运算] 🚀 模块{mid} Arm{arm_idx + 1} 摆动增量 {msg.position[0]:.2f}°")
            time.sleep(8.0)

            # 伸缩阶段
            for arm_idx in sorted(arms.keys()):
                msg = arms[arm_idx]["extend"]
                msg.header.stamp = self.get_clock().now().to_msg()
                if hasattr(msg, 'position') and len(msg.position) > 0 and msg.position[0] == 0:
                    continue
                self.pub_tel_seq.publish(msg)
                self.get_logger().info(f"[逆运算] 🚀 模块{mid} Arm{arm_idx+1} 伸缩增量 {msg.position[0]:.1f}mm")

            self._schedule_pressure(mid)
        except Exception as e:
            self.get_logger().error(f"模块{mid} 顺序执行异常: {e}")

    # ================= 第二套功能：前端微调转发 =================
    def cb_adjust_rotation(self, msg):
        self._handle_adjust(msg, "rotate")

    def cb_adjust_swing(self, msg):
        self._handle_adjust(msg, "swing")

    def cb_adjust_telescopic(self, msg):
        self._handle_adjust(msg, "extend")
    def _handle_adjust(self, msg, joint_name):
        """将前端微调指令立即转发到对应关节的功能节点，并启动7秒压力触发定时器"""
        try:
            device_id = msg.device_id
            arm_idx, _ = self._parse_device(device_id)
            if arm_idx < 0 or arm_idx > 2:
                self.get_logger().error(f"device_id {device_id} 非法")
                return

            module_id = msg.module_id
            msg.header.stamp = self.get_clock().now().to_msg()

            # 直接发布到共用的话题（与逆解下发话题相同）
            if joint_name == "rotate":
                self.pub_rot_seq.publish(msg)
            elif joint_name == "swing":
                self.pub_sw_seq.publish(msg)
            elif joint_name == "extend":
                self.pub_tel_seq.publish(msg)
            else:
                self.get_logger().error(f"未知关节类型 {joint_name}")
                return

            self.get_logger().info(f"[微调] 🎯 Arm{arm_idx+1} {joint_name} -> {msg.position[0]:.2f}")
            self._schedule_pressure(module_id)
        except Exception as e:
            self.get_logger().error(f"前端调节异常: {e}")


def main(args=None):
    rclpy.init(args=args)
    controller = ArmSequenceController()

    # 核心：多线程执行器适配时序回调
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
