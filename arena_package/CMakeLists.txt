cmake_minimum_required(VERSION 3.8)
project(arena_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)

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


add_executable(supervisor_node src/SupervisorNode.cpp)

ament_target_dependencies(supervisor_node
  rclcpp
  std_msgs
  nav_msgs
)

target_link_libraries(supervisor_node ${OpenCV_LIBS})



add_executable(arena_calibrator src/ArenaCalibration.cpp)

ament_target_dependencies(arena_calibrator
  rclcpp
  sensor_msgs
  cv_bridge
  ament_index_cpp
)

target_link_libraries(arena_calibrator ${OpenCV_LIBS})

install(TARGETS
  supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
