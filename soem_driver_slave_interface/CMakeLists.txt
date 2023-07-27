cmake_minimum_required(VERSION 3.12)
project(soem_driver_slave_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(soem_driver_common REQUIRED)

ament_export_include_directories(
  include
)

add_library(
  ${PROJECT_NAME}
  INTERFACE
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE
  hardware_interface
  rclcpp
  # soem_driver
  soem_driver_common
)

# ament_target_dependencies(
#   ${PROJECT_NAME}
#   hardware_interface
#   pluginlib
#   rclcpp
# )

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include
)

## TESTING
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_dependencies(
  rclcpp
  hardware_interface
  soem_driver_common
)
ament_package()
