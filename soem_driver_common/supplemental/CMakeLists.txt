# add all sub directories and let them handle CMake themselves
file(GLOB SUPPLEMENTAL_GLOB LIST_DIRECTORIES true "${CMAKE_CURRENT_LIST_DIR}/*")
foreach(S ${SUPPLEMENTAL_GLOB})
	if(IS_DIRECTORY ${S})
		# add_subdirectory(${S})
		include(${S}/CMakeLists.txt)
	endif()
endforeach()