cmake_minimum_required(VERSION 3.8)
project(caster_swerve_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  generate_parameter_library
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  sensor_msgs
  visualization_msgs
  rcpputils
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

generate_parameter_library(caster_swerve_drive_controller_parameter
  config/caster_swerve_drive_controller_parameter.yaml
)

add_library(caster_swerve_drive_controller
  SHARED
  src/caster_swerve_drive_controller.cpp
)
target_compile_features(caster_swerve_drive_controller PUBLIC cxx_std_17)  # Require C++17

target_include_directories(caster_swerve_drive_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/caster_swerve_drive_controller>
)

target_link_libraries(caster_swerve_drive_controller PUBLIC
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  ${generate_parameter_library_TARGETS}
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  ${tf2_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  rcpputils::rcpputils
  caster_swerve_drive_controller_parameter
)

# Export the plugin definition file
pluginlib_export_plugin_description_file(controller_interface caster_swerve_drive_controller.xml) # Use your XML filename

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
  DESTINATION include/caster_swerve_drive_controller
)
install(
  TARGETS caster_swerve_drive_controller caster_swerve_drive_controller_parameter
  EXPORT export_caster_swerve_drive_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(
  export_caster_swerve_drive_controller HAS_LIBRARY_TARGET
)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
