cmake_minimum_required(VERSION 3.8)
project(soem LANGUAGES C)


# # checkout SOEM submodule if required
# if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/soem/soem)
#     message(STATUS "SOEM not found; fetching")
#     find_package(Git QUIET)
#     if(GIT_FOUND)
#     # Update submodules as needed
#         option(CHECKOUT_SOEM_SUBMODULE "Checkout SOEM submodule during build" ON)
#         if(CHECKOUT_SOEM_SUBMODULE)
#             message(STATUS "fetching SOEM submodule")
#             execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
#                             WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#                             RESULT_VARIABLE CHECKOUT_SOEM_SUBMODULE_RESULT)
#             if(NOT CHECKOUT_SOEM_SUBMODULE_RESULT EQUAL "0")
#                 message(FATAL_ERROR "git submodule update --init --recursive failed with ${CHECKOUT_SOEM_SUBMODULE_RESULT}, please checkout submodules")
#             endif()
#         endif()
#     endif()

#     if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/soem/soem)
#         message(FATAL_ERROR "SOEM submodule was not downloaded! CHECKOUT_SOEM_SUBMODULE was turned off or failed. Please update submodules and try again.")
#     endif()
# endif()


find_package(Threads REQUIRED)


set(SOEM_OS "linux")


file(GLOB SOEM_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/soem/soem/*.c)
file(GLOB OSAL_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/soem/osal/${SOEM_OS}/*.c)
file(GLOB OSHW_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/soem/oshw/${SOEM_OS}/*.c)

add_library(soem STATIC
  ${SOEM_SOURCES}
  ${OSAL_SOURCES}
  ${OSHW_SOURCES}
  ${OSHW_EXTRA_SOURCES}
)

target_link_libraries(soem PRIVATE Threads::Threads rt)


file(GLOB SOEM_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/soem/soem/*.h)
file(GLOB OSAL_HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/soem/osal/osal.h
  ${CMAKE_CURRENT_SOURCE_DIR}/soem/osal/${SOEM_OS}/*.h
)
file(GLOB OSHW_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/soem/oshw/${SOEM_OS}/*.h)
file(
  COPY ${SOEM_HEADERS} ${OSAL_HEADERS} ${OSHW_HEADERS}
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include/soem
)

# classify as SYSTEM includes to disable pedandic warnings for included SOEM library
# find copied headers from build space during build
target_include_directories(soem SYSTEM PUBLIC
  # make application find installed headers under "soem/*.h"
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # make SOEM headers find themselves after install
  $<INSTALL_INTERFACE:include/soem>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include/soem>
)

# treat everything as errors, but do not be pedantic with SOEM
set(SOEM_C_FLAGS -Wall -Wextra -Werror) # -fPIC)
target_compile_options(soem PRIVATE ${SOEM_C_FLAGS})


# install SOEM
install(
  TARGETS soem
  DESTINATION lib
)
install(FILES
  ${SOEM_HEADERS}
  ${OSAL_HEADERS}
  ${OSHW_HEADERS}
  DESTINATION include/soem
)