cmake_minimum_required(VERSION 3.22)
project(lite_moveit)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/moveit_remote_policy_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME moveit_remote_policy_bridge
)

# 力矩补偿模块 — 放在 lib/ 下供 bridge 节点 import
install(
  PROGRAMS scripts/torque_compensation.py
  DESTINATION lib/${PROJECT_NAME}
)

# 电机状态监控仪表盘
install(
  PROGRAMS scripts/motor_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motor_monitor
)