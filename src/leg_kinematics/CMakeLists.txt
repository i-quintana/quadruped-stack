cmake_minimum_required(VERSION 3.8)
project(leg_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# The kinematics core: header-only, Eigen only, no ROS.
add_library(leg_kinematics INTERFACE)
target_compile_features(leg_kinematics INTERFACE cxx_std_17)

target_include_directories(leg_kinematics INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(leg_kinematics INTERFACE Eigen3::Eigen)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS leg_kinematics EXPORT export_leg_kinematics)

ament_export_targets(export_leg_kinematics HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

add_executable(foot_position_node src/foot_position_node.cpp)
target_link_libraries(foot_position_node leg_kinematics)
ament_target_dependencies(foot_position_node
  rclcpp sensor_msgs geometry_msgs tf2_ros)

install(TARGETS foot_position_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()