cmake_minimum_required(VERSION 3.8)
project(tracker_rgb_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tracker_package REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(visualization_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)




# Incluir explícitamente ArUco si la versión de OpenCV lo requiere
# En muchas instalaciones de ROS, está dentro de OpenCV_LIBS
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(tracker_node_RGB src/TrackerNodeRGB.cpp)

# IMPORTANTE: Asegúrate de que cv_bridge y OpenCV estén vinculados
ament_target_dependencies(tracker_node_RGB
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

target_link_libraries(tracker_node_RGB 
  ${OpenCV_LIBS}
)

add_executable(transformer_node_RGB src/TransformerNodeRGB.cpp)

ament_target_dependencies(transformer_node_RGB
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tracker_package
  visualization_msgs
)


target_link_libraries(transformer_node_RGB
  ${cpp_typesupport_target}
)



install(TARGETS
  tracker_node_RGB
  transformer_node_RGB
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


# Instalación de archivos de launch o configuración si los tuvieras
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()