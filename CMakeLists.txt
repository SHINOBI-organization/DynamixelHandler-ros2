cmake_minimum_required(VERSION 3.8)
project(dynamixel_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED) # 自作msgの作成のために必要
find_package(std_msgs REQUIRED) # 自作msgのHeaderのために必要

# 以下の部分は自作ｍｓｇの定義のために必要
set(msg_files
  # read専用
  "msg/DynamixelState.msg"
  "msg/DynamixelError.msg"

  # read/write
  "msg/DynamixelConfig.msg"
  "msg/DynamixelExtra.msg"
  "msg/DynamixelGain.msg"
  "msg/DynamixelGoal.msg"
  "msg/DynamixelLimit.msg"
  "msg/DynamixelMode.msg"
    
  # write専用
  "msg/DynamixelCommand.msg"
  "msg/DynamixelCommandXControlCurrent.msg"
  "msg/DynamixelCommandXControlCurrentPosition.msg"
  "msg/DynamixelCommandXControlExtendedPosition.msg"
  "msg/DynamixelCommandXControlPosition.msg"
  "msg/DynamixelCommandXControlVelocity.msg"
  "msg/DynamixelCommandPControlCurrent.msg"
  "msg/DynamixelCommandPControlExtendedPosition.msg"
  "msg/DynamixelCommandPControlPosition.msg"
  "msg/DynamixelCommandPControlVelocity.msg"
)
set(srv_files
# rebootとかはこっちかな？
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
## ここまで

include_directories(
  include/mylib_dynamixel/src
  ${catkin_INCLUDE_DIRS}
)

# ===============================dynamixel_unify_baudrate_node===============================

# ノードの定義
add_executable(
  dynamixel_unify_baudrate_node 
	include/mylib_dynamixel/download/port_handler_linux_fix.cpp
	include/mylib_dynamixel/download/port_handler_fix.cpp
	include/mylib_dynamixel/src/dynamixel_communicator_main.cpp
	include/mylib_dynamixel/src/dynamixel_communicator_retry.cpp
  
	src/main_dynamixel_unify_baudrate.cpp
)
target_include_directories(
  dynamixel_unify_baudrate_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
target_compile_features(
  dynamixel_unify_baudrate_node 
  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
	dynamixel_unify_baudrate_node
  "rclcpp"
  "rosidl_default_generators"
)

install(
  TARGETS dynamixel_unify_baudrate_node
  DESTINATION lib/${PROJECT_NAME}
  )

# ここまで

# 以下の部分は同じPkg内でmsgを定義＆使用を行う場合にい必要な追記
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(dynamixel_unify_baudrate_node "${cpp_typesupport_target}")

# ============================dynamixel_handler==================================

# ノードの定義
add_executable(
  dynamixel_handler_node 
	include/mylib_dynamixel/download/port_handler_linux_fix.cpp
	include/mylib_dynamixel/download/port_handler_fix.cpp
	include/mylib_dynamixel/src/dynamixel_communicator_main.cpp
	include/mylib_dynamixel/src/dynamixel_communicator_retry.cpp
  
	src/dynamixel_handler_ros_interface.cpp
	src/dynamixel_handler_dyn_interface_loop.cpp
	src/dynamixel_handler_dyn_interface_once.cpp
	src/main_dynamixel_handler.cpp
)
target_include_directories(
  dynamixel_handler_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
target_compile_features(
  dynamixel_handler_node 
  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
	dynamixel_handler_node
  "rclcpp"
  "rosidl_default_generators"
)

install(
  TARGETS dynamixel_handler_node
  DESTINATION lib/${PROJECT_NAME}
)
# ここまで

# 以下の部分は同じPkg内でmsgを定義＆使用を行う場合にい必要な追記
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(dynamixel_handler_node "${cpp_typesupport_target}")
# ==================================================================================

## ここまで

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
