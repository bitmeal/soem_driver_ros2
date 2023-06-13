cmake_minimum_required(VERSION 3.8)
project(soem_modules_mock_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(soem_slave_interface REQUIRED)
# find_package(soem_driver REQUIRED)


file(GLOB_RECURSE PLUGINS_SRC src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${PLUGINS_SRC})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  pluginlib
  soem_slave_interface
  # soem_driver
)


pluginlib_export_plugin_description_file(soem_slave_interface soem_slave_plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
  find_package(pluginlib REQUIRED)
  # find_package(hardware_interface REQUIRED) # already satisfied

  # test loading of driver plugin by resource manager
  ament_add_gmock(test_load_soem_mock_module test/test_load_soem_mock_module.cpp)
  ament_target_dependencies(test_load_soem_mock_module
    pluginlib
    soem_slave_interface
  )
  set_property(TARGET test_load_soem_mock_module PROPERTY CXX_STANDARD 20)

  # test basic behavior
  ament_add_gmock(test_soem_mock_module test/test_soem_mock_module.cpp)
  ament_target_dependencies(test_soem_mock_module
    rclcpp
    pluginlib
    soem_slave_interface
  )
  set_property(TARGET test_soem_mock_module PROPERTY CXX_STANDARD 20)

endif()


ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
