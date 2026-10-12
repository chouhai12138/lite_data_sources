#!/usr/bin/env bash
# 编译 /data/lite_data_sources（lite_moveit），并让它对接 /data/lite_motor_ws 的 lite_bridge。
#
# 两个关键点（踩过的坑）：
#   1. PATH 必须把 /usr/bin 放最前。
#      用户级 uv Python `/home/chouhai/.local/bin/python3.12` 会抢先被 CMake 的
#      find_package(Python3) 选中，而它没装 catkin_pkg，构建会在
#      `ament_package_xml.cmake` 处直接报 ModuleNotFoundError: No module named 'catkin_pkg'。
#   2. 必须先 source /data/lite_motor_ws/install/setup.bash。
#      lite_moveit 依赖该 underlay 提供 lite_bridge / lite_bringup /
#      lite_controller / robstride_msgs / lite_description。
set -euo pipefail

export PATH=/usr/bin:$PATH
export ROS_HOME="${ROS_HOME:-$HOME/.ros}"

source /opt/ros/jazzy/setup.bash
source /data/lite_motor_ws/install/setup.bash

cd "$(dirname "$(readlink -f "$0")")"
colcon build --symlink-install "$@"

echo
echo "编译完成。使用时请依次 source 两个工作区："
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source /data/lite_motor_ws/install/setup.bash"
echo "  source /data/lite_data_sources/install/setup.bash"
echo "  ros2 launch lite_moveit one_click.launch.py use_fake_hardware:=true   # 空跑"
echo "  ros2 launch lite_moveit one_click.launch.py                          # 真机(停在 HOLD)"
