cmake_minimum_required(VERSION 3.12)
project(soem_driver_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###############################
# REPO AUTOMATION
function(checkout_submodules)
  if(NOT SUBMODULE_CHECKOUT_FAILED)
    # reset on success  
    # set(SUBMODULE_CHECKOUT_FAILED False CACHE BOOL "indicating automatic submodule checkout failure")

    find_package(Git REQUIRED)

    # find repo root
    execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse --show-toplevel
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                    RESULT_VARIABLE GIT_REPO_ROOT_RESULT
                    OUTPUT_VARIABLE GIT_REPO_ROOT)
    if(GIT_REPO_ROOT_RESULT EQUAL "0")

      string(STRIP ${GIT_REPO_ROOT} GIT_REPO_ROOT)

      # find submodule path config keys
      execute_process(COMMAND ${GIT_EXECUTABLE} config --file "${GIT_REPO_ROOT}/.gitmodules" --name-only --get-regexp "path"
                      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                      RESULT_VARIABLE GIT_SUBMODULES_PATH_CFG_KEYS_RESULT
                      OUTPUT_VARIABLE GIT_SUBMODULES_PATH_CFG_KEYS)
      if(GIT_SUBMODULES_PATH_CFG_KEYS_RESULT EQUAL "0")

        # clean config keys
        string(STRIP GIT_SUBMODULES_PATH_CFG_KEYS ${GIT_SUBMODULES_PATH_CFG_KEYS})
        string(REPLACE "\n" ";" GIT_SUBMODULES_PATH_CFG_KEYS ${GIT_SUBMODULES_PATH_CFG_KEYS})

        # get paths from config keys and check out submodule if needed
        foreach(SUBMODULE_PATH_CFG_KEY IN LISTS GIT_SUBMODULES_PATH_CFG_KEYS)
          # skip empty strings
          if(NOT SUBMODULE_PATH_CFG_KEY)
            continue()
          endif()
          
          execute_process(COMMAND ${GIT_EXECUTABLE} config --file "${GIT_REPO_ROOT}/.gitmodules" --get "${SUBMODULE_PATH_CFG_KEY}"
                          WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                          RESULT_VARIABLE GIT_SUBMODULE_PATH_RESULT
                          OUTPUT_VARIABLE GIT_SUBMODULE_PATH)
          if(NOT GIT_SUBMODULE_PATH_RESULT EQUAL "0")
              # message(FATAL_ERROR "could not get submodules path from config; please make sure to checkout submodules manually")
            set(SUBMODULE_CHECKOUT_FAILED True CACHE BOOL "indicating automatic submodule checkout failure")
            continue()
          endif()

          string(REPLACE "\n" "" GIT_SUBMODULE_PATH ${GIT_SUBMODULE_PATH})
          string(STRIP GIT_SUBMODULE_PATH ${GIT_SUBMODULE_PATH})
          set(GIT_SUBMODULE_FULL_PATH "${GIT_REPO_ROOT}/${GIT_SUBMODULE_PATH}")
          set(GIT_SUBMODULE_GIT_FS_ENTRY "${GIT_SUBMODULE_FULL_PATH}/.git")
          
          # if (EXISTS ${GIT_SUBMODULE_GIT_FILE} AND NOT IS_DIRECTORY ${GIT_SUBMODULE_GIT_FILE})
          if (NOT EXISTS ${GIT_SUBMODULE_GIT_FS_ENTRY})
            # is not checked out: no .git file or directory
            message(STATUS "checking out submodule in ${GIT_SUBMODULE_PATH}")
            execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive ${GIT_SUBMODULE_FULL_PATH}
                            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                            RESULT_VARIABLE CHECKOUT_SUBMODULE_RESULT)
            if(NOT CHECKOUT_SUBMODULE_RESULT EQUAL "0")
                # message(FATAL_ERROR "failed to check out submodule at ${GIT_SUBMODULE_FULL_PATH}; please checkout submodules")
                set(SUBMODULE_CHECKOUT_FAILED True CACHE BOOL "indicating automatic submodule checkout failure")
            endif()
          elseif(IS_DIRECTORY ${GIT_SUBMODULE_GIT_FS_ENTRY} AND NOT SUBMODULE_WARNING_ISSUED)
            # has .git directory:
            # - repo exists
            # - has been checked out by other tool or manually
            message(WARNING "${GIT_SUBMODULE_PATH} is no submodule! You are fine for building only!\nFor development of this package please use as submodule!")
            set(SUBMODULE_WARNING_ISSUED True CACHE BOOL "you have been informed of at least one submodule that has not been checked out as submodule; disables future warnings")
          else()
            # has .git file: is submodule and checked out OK
            message(STATUS "${GIT_SUBMODULE_PATH} submodule OK")
          endif()
        endforeach()


      else()
        # message(WARNING "could not find submodules config paths; please make sure to checkout submodules manually")
        set(SUBMODULE_CHECKOUT_FAILED True CACHE BOOL "indicating automatic submodule checkout failure")
      endif()

    else()
      # message(WARNING "could not find repository root; please make sure to check out submodules manually")
      set(SUBMODULE_CHECKOUT_FAILED True CACHE BOOL "indicating automatic submodule checkout failure")
    endif()

  else()
    # error already present from previous try; skip
    return()
  endif()

  # message if error in this run
  if(SUBMODULE_CHECKOUT_FAILED)
    message(WARNING "checking out submodules failed! please check out submodules manually. will NOT RETRY!")
  endif()

  mark_as_advanced(FORCE SUBMODULE_CHECKOUT_FAILED)
endfunction()

checkout_submodules()


###############################
# building and providing code
# 
# find dependencies
find_package(ament_cmake REQUIRED)

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
ament_export_dependencies()
ament_package()
