cmake_minimum_required(VERSION 3.16)
project(caster_swerve_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  generate_parameter_library
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2_geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(caster_swerve_drive_controller_parameter
  src/caster_swerve_drive_controller_parameter.yaml
)

add_library(caster_swerve_drive_controller
  SHARED
  src/caster_swerve_drive_controller.cpp
)
target_compile_features(caster_swerve_drive_controller PUBLIC cxx_std_17)  # Require C++17
target_include_directories(caster_swerve_drive_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(caster_swerve_drive_controller PUBLIC
  caster_swerve_drive_controller_parameter
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${geometry_msgs_TARGETS}
)


if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_caster_swerve_drive_controller
    test/test_caster_swerve_drive_controller.cpp
  )
  target_link_libraries(test_caster_swerve_drive_controller
    caster_swerve_drive_controller
  )

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_caster_swerve_drive_controller test/test_load_caster_swerve_drive_controller.cpp)
  target_link_libraries(test_load_caster_swerve_drive_controller
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS caster_swerve_drive_controller caster_swerve_drive_controller_parameter
  EXPORT export_caster_swerve_drive_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(controller_interface caster_swerve_drive_controller.xml)
ament_export_targets(
  export_caster_swerve_drive_controller HAS_LIBRARY_TARGET
)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
