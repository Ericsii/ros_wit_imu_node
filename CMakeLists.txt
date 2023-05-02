cmake_minimum_required(VERSION 3.8)
project(wit_imu_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
)

include_directories(include)

add_executable(
  wit_imu_node
  "src/wt901c.cpp"
  "src/wit_imu_driver_node.cpp"
)

ament_target_dependencies(wit_imu_node ${dependencies})

install(
  TARGETS wit_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
