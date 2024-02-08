# add all sub directories and let them handle CMake themselves
file(GLOB SUPPLEMENTAL_GLOB LIST_DIRECTORIES true "*")
foreach(S ${SUPPLEMENTAL_GLOB})
	if(IS_DIRECTORY ${S})
		add_subdirectory(${S})
	endif()
endforeach()