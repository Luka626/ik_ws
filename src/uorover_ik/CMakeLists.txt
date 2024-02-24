cmake_minimum_required(VERSION 3.8)
project(uorover_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(general_interfaces REQUIRED)

add_executable(uorover_ik src/uorover_ik.cpp)
add_executable(ik_hardware_interface src/ik_hardware_interface.cpp)

target_include_directories(uorover_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(uorover_ik PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  uorover_ik
  "moveit_ros_planning_interface"
  "rclcpp"
  "general_interfaces"
)

ament_target_dependencies(ik_hardware_interface "moveit_ros_planning_interface" "rclcpp" "std_msgs" "control_msgs" "general_interfaces")

install(TARGETS uorover_ik ik_hardware_interface
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
