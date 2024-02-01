cmake_minimum_required(VERSION 3.8)
project(soem_modules_trinamic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(soem_driver_slave_interface REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)


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
  soem_driver_slave_interface
  yaml_cpp_vendor
)


pluginlib_export_plugin_description_file(soem_driver_slave_interface soem_slave_plugins.xml)

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

  # build one test suite from all tests
  file(GLOB_RECURSE PLUGINS_TESTS_SRC test/test_*.cpp)

  ament_add_gmock(test_soem_modules_trinamic ${PLUGINS_TESTS_SRC})
  ament_target_dependencies(test_soem_modules_trinamic
    pluginlib
    soem_driver_slave_interface
  )
  set_property(TARGET test_soem_modules_trinamic PROPERTY CXX_STANDARD 20)

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
