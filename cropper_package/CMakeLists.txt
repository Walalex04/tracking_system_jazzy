cmake_minimum_required(VERSION 3.8)
project(cropper_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# =========================
# NODO 1: cropper
# =========================
add_executable(cropper_node src/CropperNode.cpp)

ament_target_dependencies(cropper_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(cropper_node
  ${OpenCV_LIBS}
)

# =========================
# NODO 2: calibrator
# =========================
add_executable(cropper_calibration src/CropperCalibration.cpp)

ament_target_dependencies(cropper_calibration
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(cropper_calibration
  ${OpenCV_LIBS}
)

# =========================
# INSTALL
# =========================
install(TARGETS
  cropper_node
  cropper_calibration
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()