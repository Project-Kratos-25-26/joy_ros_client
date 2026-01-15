cmake_minimum_required(VERSION 3.8)
project(joy_ros_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

add_executable(gimblectl gimblectl/camctl2.c)
add_executable(serial_client gimblectl/serial_client.c)
target_link_libraries(gimblectl m)
add_executable(joy_ros_client src/main.cpp)
add_executable(testing src/subtest.cpp)
ament_target_dependencies(joy_ros_client rclcpp std_msgs)
ament_target_dependencies(testing rclcpp std_msgs)
install(TARGETS
  joy_ros_client
  testing
  gimblectl
  serial_client
  DESTINATION lib/${PROJECT_NAME})


ament_package()
