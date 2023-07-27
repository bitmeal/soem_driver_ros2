cmake_minimum_required(VERSION 3.8)
project(soem_driver_ros2 NONE)
find_package(ament_cmake REQUIRED)
ament_package()

find_package(Git REQUIRED)

# find all submodules
execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
RESULT_VARIABLE CHECKOUT_SOEM_SUBMODULE_RESULT)
if(NOT CHECKOUT_SOEM_SUBMODULE_RESULT EQUAL "0")
message(FATAL_ERROR "git submodule update --init --recursive failed with ${CHECKOUT_SOEM_SUBMODULE_RESULT}, please checkout submodules")
endif()


# checkout all submodules
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/soem/soem)
    message(STATUS "SOEM not found; fetching")
    find_package(Git QUIET)
    if(GIT_FOUND)
    # Update submodules as needed
        option(CHECKOUT_SOEM_SUBMODULE "Checkout SOEM submodule during build" ON)
        if(CHECKOUT_SOEM_SUBMODULE)
            message(STATUS "fetching SOEM submodule")
            execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                            RESULT_VARIABLE CHECKOUT_SOEM_SUBMODULE_RESULT)
            if(NOT CHECKOUT_SOEM_SUBMODULE_RESULT EQUAL "0")
                message(FATAL_ERROR "git submodule update --init --recursive failed with ${CHECKOUT_SOEM_SUBMODULE_RESULT}, please checkout submodules")
            endif()
        endif()
    endif()

    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/soem/soem)
        message(FATAL_ERROR "SOEM submodule was not downloaded! CHECKOUT_SOEM_SUBMODULE was turned off or failed. Please update submodules and try again.")
    endif()
endif()

