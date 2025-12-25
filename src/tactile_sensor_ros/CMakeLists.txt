cmake_minimum_required(VERSION 3.8)
project(tactile_sensor_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# --- 1. 生成自定义消息 (保留包名 tactile_sensor_ros) ---
# 这会生成 tactile_sensor_ros.msg 模块
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FingerData.msg"
  "msg/TactileArray.msg"
  DEPENDENCIES std_msgs
)

# --- 2. 安装你的 Python 驱动代码 ---
# 注意：这里改成了 tactile_driver
ament_python_install_package(tactile_driver)

# --- 3. 安装 Python 节点入口脚本 ---
# 注意：路径变成了 tactile_driver/tactile_node.py
install(PROGRAMS
  tactile_driver/tactile_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- 4. 安装 Launch 文件 ---
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --- 5. 安装库文件 (.so) ---
install(DIRECTORY lib
  DESTINATION share/${PROJECT_NAME}
)

ament_package()