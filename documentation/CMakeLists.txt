
# ---------- DOXYGEN ---------- #

# Find the installed doxygen executable
find_package(Doxygen REQUIRED)
find_package(Git QUIET)

# ---- Make sure to recursively clone all the git submodules for external libraries (doxygen-awesome) --- #
if(GIT_FOUND)
    if (EXISTS "${CMAKE_SOURCE_DIR}/.git")
    	# Update submodules as needed
	option(CLONE_DOCUMENTATION_GIT_SUBMODULES "Check submodules during build" ON)
    	if(CLONE_DOCUMENTATION_GIT_SUBMODULES)
		message(STATUS "Documentation Git Submodules update")
		message(STATUS "Working directory ${CMAKE_CURRENT_SOURCE_DIR}")
        	execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        	RESULT_VARIABLE GIT_SUBMOD_RESULT)
        	if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            	    message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        	endif()
    	endif()
    else()
  	message(FATAL_ERROR "This is not a Git repository. In order to build the documentation, you need to clone the ReactPhysics3D repository.")
    endif()
else()
  message(FATAL_ERROR "Git has not been found on your system. This is necessary to build the documentation because git submodules command is used to get the documentation dependencies (doxygen-awesome library). You need to fill in the GIT_EXECUTABLE CMake variable with the path to the Git executable on your system to continue.")
endif()

# ---------- Doxygen --------- #

# Set variables
set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/html/index.html)
set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
set(DOXYFILE_OUT ${DOXYGEN_OUTPUT_DIR}/Doxyfile)

# Replace variables inside @@ with the current values in the Doxyfile.in and export to Doxyfile
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

# Copy the "doxygen-awesome-css" folder to build directory
file(COPY doxygen-awesome-css DESTINATION ${DOXYGEN_OUTPUT_DIR})

# Copy the "images" folder to build directory
file(COPY images DESTINATION ${DOXYGEN_OUTPUT_DIR}/html)

# Create the Doxygen output directory
file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR})
add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
                   COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
                   MAIN_DEPENDENCY ${DOXYFILE_OUT} ${DOXYFILE_IN}
		   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/UserDocumentation.md
		   COMMENT "Generating documentation with Doxygen")

add_custom_target(Doxygen ALL DEPENDS ${DOXYGEN_INDEX_FILE})
