cmake_minimum_required(VERSION 3.5)
project(robot_patrol)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robot_patrol REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToPoint.action"
  "srv/GetDirection.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

add_executable(start_patrolling src/patrol.cpp)
ament_target_dependencies(start_patrolling rclcpp nav_msgs rclcpp_action 
geometry_msgs std_msgs  sensor_msgs robot_patrol)


add_executable(direction_server src/direction_service.cpp)
ament_target_dependencies(direction_server rclcpp std_msgs  sensor_msgs robot_patrol)

add_executable(client_server src/test_service.cpp)
ament_target_dependencies(client_server rclcpp nav_msgs geometry_msgs std_msgs sensor_msgs robot_patrol)

add_executable(patrol_v2 src/patrol_v2.cpp)
ament_target_dependencies(patrol_v2 rclcpp nav_msgs rclcpp_action 
geometry_msgs std_msgs  sensor_msgs robot_patrol)

add_executable(goToPoint src/go_to_point_action.cpp)
ament_target_dependencies(goToPoint rclcpp_action  rclcpp nav_msgs geometry_msgs std_msgs sensor_msgs robot_patrol)


# Install executable files
install(TARGETS
   start_patrolling
   direction_server
   client_server
   patrol_v2
   goToPoint
   DESTINATION lib/${PROJECT_NAME}/
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
