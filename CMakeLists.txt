cmake_minimum_required(VERSION 3.8)
project(pecka_tvmc_msg)

# Core build system
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message dependencies
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# List message files
set(msg_files
  "msg/Command.msg"
  "msg/ControlMode.msg"
  "msg/CurrentPoint.msg"
  "msg/DoF.msg"
  "msg/PidConstants.msg"
  "msg/PidLimits.msg"
  "msg/TargetPoint.msg"
  "msg/Thrust.msg"
  "msg/LEDControl.msg"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_python_install_package(tvmc PACKAGE_DIR src/tvmc)

# Export runtime deps so other packages can use the messages
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs geometry_msgs)

file(GLOB ALL_SCRIPTS "scripts/others/*.py")
install(PROGRAMS
  ${ALL_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
