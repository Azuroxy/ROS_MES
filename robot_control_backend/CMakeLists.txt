cmake_minimum_required(VERSION 3.8)
project(robot_control_backend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RotationCmd.msg"
  "msg/SwingCmd.msg"
  "msg/Feedback.msg"
  "msg/TelescopicCmd.msg"
  "msg/SensorCmd.msg"
  "msg/Kinematics.msg"
  "msg/IntCmd.msg"
  "msg/GyroFeedback.msg"
  "msg/TuoLuoYi.msg"
  "msg/Heartbeat.msg"
  "msg/AxisStatus.msg"
  "msg/ArriveNotification.msg"
  "msg/StopNotification.msg"
  "msg/CmdAck.msg"
  "msg/HomeStatus.msg"
  "msg/AxisFault.msg"
  "msg/LimitEvent.msg"
  "msg/VacuumLoss.msg"
  "msg/UidAnnounce.msg"
  "msg/SegAssign.msg"
  "msg/SegConfirm.msg"
  "msg/ModuleStatus.msg"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  scripts/kinematics_node.py
  scripts/calculation_node.py
  scripts/feedback_node.py
  scripts/module_confirme_node.py
  scripts/data_process_node.py
  scripts/sensor_control_node.py
  scripts/rotation_node.py
  scripts/swing_node.py
  scripts/telescopic_node.py
  scripts/hardware_node.py
  scripts/zlg_bus.py
  scripts/sim_lower_machine.py
  scripts/rosbridge_wrapper.py
  scripts/softstop_node.py
  scripts/control_node.py
  scripts/imu_average_calculator_node.py
  scripts/tuo_luo_yi.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES rob_arm.env
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
