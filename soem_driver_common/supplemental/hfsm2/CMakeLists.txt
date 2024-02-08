# provide HFSM2 library

find_package(ament_cmake REQUIRED)

add_library(
  hfsm2
  INTERFACE
)

set_property(TARGET hfsm2 PROPERTY CXX_STANDARD 20)

target_include_directories(hfsm2 INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/hfsm2/include>
  $<INSTALL_INTERFACE:include>
)


# INSTALL
install(
  DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/hfsm2/include/
  DESTINATION include
)

## EXPORTS
ament_export_include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/hfsm2/include
)
