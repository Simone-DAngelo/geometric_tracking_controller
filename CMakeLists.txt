cmake_minimum_required(VERSION 3.8)
project(geometric_tracking_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

# Include Eigen3
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(geometric_tracking_controller src/main.cpp src/controller/lee_controller.cpp)
ament_target_dependencies(geometric_tracking_controller 
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  std_msgs 
  actuator_msgs
  Eigen3
  Boost
)

install(TARGETS
  geometric_tracking_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
