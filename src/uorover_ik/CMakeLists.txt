
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
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
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(uorover_ik src/uorover_ik.cpp)

add_library(
    rmm_hardware
    SHARED
    hardware/rmm_hardware.cpp
)

target_include_directories(uorover_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(uorover_ik PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  uorover_ik
  "moveit_ros_planning_interface"
  "rclcpp"
  "general_interfaces"
)

ament_target_dependencies(
  rmm_hardware PUBLIC
  "pluginlib"
  "rclcpp_lifecycle"
  "hardware_interface"
  "general_interfaces"
  "rclcpp"
  )

pluginlib_export_plugin_description_file(hardware_interface rmm_hardware.xml)

install(TARGETS uorover_ik 
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(
    DIRECTORY hardware/include/
    DESTINATION include/rmm_hardware
)

install(TARGETS rmm_hardware
    EXPORT rmm_hardware
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
)

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

ament_export_targets(rmm_hardware HAS_LIBRARY_TARGET)
ament_package()
