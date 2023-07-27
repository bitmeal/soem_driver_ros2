cmake_minimum_required(VERSION 3.12)
project(soem_ethercat_grant LANGUAGES C)

# add default compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CXX_FLAGS -Wall -Wextra -Wpedantic)
  add_compile_options(${CXX_FLAGS})
  # # add to C++ targets only, not C
  # add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:${CXX_FLAGS}>")
endif()


find_package(ament_cmake QUIET)
# find_package(procps REQUIRED)
# find_package(cap-ng REQUIRED)

add_executable(${PROJECT_NAME} cap-grant-ld/cap-grant-ld/cap_grant_ld.c)
target_link_libraries(${PROJECT_NAME} procps cap-ng)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS postinst.bash
  DESTINATION lib/${PROJECT_NAME}
  RENAME postinst
)

# ## TESTING
# if(BUILD_TESTING)
#     # TODO
# endif()

ament_package()
