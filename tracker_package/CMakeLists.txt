cmake_minimum_required(VERSION 3.8)
project(tracker_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# DEPENDENCIAS
# =========================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# =========================
# MENSAJE PERSONALIZADO
# =========================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotInfo.msg"
  DEPENDENCIES std_msgs nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

  
# =========================
# NODO 1: TRACKER
# =========================
add_executable(tracker_node src/TrackerNode.cpp)

ament_target_dependencies(tracker_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

target_link_libraries(tracker_node
  ${OpenCV_LIBS}
)

# =========================
# NODO 2: TRANSFORMER
# =========================
add_executable(transformer_node src/TransformerNode.cpp)

ament_target_dependencies(transformer_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)


target_link_libraries(transformer_node
  ${cpp_typesupport_target}
)


add_executable(transformer_calibrator src/TransformerCalibrator.cpp)

ament_target_dependencies(transformer_calibrator
  rclcpp
  std_msgs
  ament_index_cpp
)

# =========================
# INSTALL
# =========================
install(TARGETS
  tracker_node
  transformer_node
  transformer_calibrator
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)



# Export para runtime de mensajes
ament_export_dependencies(rosidl_default_runtime)

ament_package()