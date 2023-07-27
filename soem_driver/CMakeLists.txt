cmake_minimum_required(VERSION 3.12)
project(soem_driver LANGUAGES CXX)

# add default compiler flags to C++ targets only, not C
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CXX_FLAGS -Wall -Wextra -Wpedantic)
  add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:${CXX_FLAGS}>")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(soem_driver_common REQUIRED)
find_package(soem_driver_slave_interface REQUIRED)

# expects soem headers to be found as "soem/*.h"
find_package(soem QUIET)
if(NOT SOEM_FOUND)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/soem)
endif()



# master
add_library(soem_master src/soem_master.cpp)
set_property(TARGET soem_master PROPERTY CXX_STANDARD 20)

target_include_directories(soem_master PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(soem_master soem)
ament_target_dependencies(
  soem_master
  rclcpp
  soem_driver_common
)


# driver
add_library(
  ${PROJECT_NAME}
  SHARED
  src/soem_driver.cpp
  # src/soem_master.cpp
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  # soem
  soem_master
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  yaml_cpp_vendor
  rclcpp
  rclcpp_lifecycle
  soem_driver_common
  soem_driver_slave_interface
)

# # Causes the visibility macros to use dllexport rather than dllimport,
# # which is appropriate when building the dll but not consuming it.
# target_compile_definitions(ethercat_driver PRIVATE "ETHERCAT_DRIVER_BUILDING_LIBRARY")
# # prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(hardware_interface soem_driver_plugin.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)


## TESTING
if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  # find_package(hardware_interface REQUIRED) # already satisfied

  # test loading of driver plugin by resource manager
  ament_add_gmock(test_load_soem_driver test/test_load_soem_driver.cpp)
  ament_target_dependencies(test_load_soem_driver
    hardware_interface
    ros2_control_test_assets
  )

  add_executable(test_soem_master test/test_soem_master.cpp)
  set_property(TARGET test_soem_master PROPERTY CXX_STANDARD 20)
  # target_link_libraries(test_soem_master soem_driver)
  target_link_libraries(test_soem_master soem_master)
  ament_target_dependencies(
    test_soem_master
    rclcpp
    # soem_driver
  )
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
  soem
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  soem_driver_common
  soem_driver_slave_interface
)

ament_package()
