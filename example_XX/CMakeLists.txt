cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_xx LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  pluginlib
  generate_parameter_library
  joint_trajectory_controller_plugins
  trajectory_msgs
  rclcpp
  rclpy
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(cartpole_lqr_trajectory_plugin_parameters
  controller/cartpole_lqr_trajectory_plugin_parameters.yaml
)
add_library(${PROJECT_NAME} SHARED controller/cartpole_lqr_trajectory_plugin.cpp)
add_library(ros2_control_demo_example_xx::cartpole_lqr_trajectory_plugin ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME} PUBLIC
  cartpole_lqr_trajectory_plugin_parameters
)
pluginlib_export_plugin_description_file(joint_trajectory_controller_plugins plugins.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_DEMO_EXAMPLE_XX_BUILDING_LIBRARY")

# INSTALL
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/gazebo
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME} cartpole_lqr_trajectory_plugin_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/trajectory_action_client.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_libraries(
  cartpole_lqr_trajectory_plugin
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
