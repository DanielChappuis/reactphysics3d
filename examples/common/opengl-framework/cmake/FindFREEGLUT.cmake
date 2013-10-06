# This module is used to try to find the Freeglut library and include files

IF(WIN32)
	FIND_PATH(FREEGLUT_INCLUDE_DIR NAMES GL/freeglut.h)
	FIND_LIBRARY(FREEGLUT_LIBRARY NAMES freeglut freeglut_static
                 PATHS ${OPENGL_LIBRARY_DIR})
ELSE(WIN32)
	
	IF(APPLE)
		# Do nothing, we do not want to use freeglut on Mac OS X
	ELSE(APPLE)
		FIND_PATH(FREEGLUT_INCLUDE_DIR GL/freeglut.h /usr/include/GL
                  /usr/openwin/share/include
				  /usr/openwin/include
                  /opt/graphics/OpenGL/include
                  /opt/graphics/OpenGL/contrib/libglut)

		FIND_LIBRARY(FREEGLUT_LIBRARY NAMES glut freeglut freeglut_static PATHS /usr/openwin/lib)
		FIND_LIBRARY(Xi_LIBRARY Xi /usr/openwin/lib)
		FIND_LIBRARY(Xmu_LIBRARY Xmu /usr/openwin/lib)
	ENDIF(APPLE)
ENDIF(WIN32)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(FREEGLUT REQUIRED_VARS FREEGLUT_LIBRARY FREEGLUT_INCLUDE_DIR)

IF(FREEGLUT_FOUND)
	SET(FREEGLUT_LIBRARIES ${FREEGLUT_LIBRARY} ${Xi_LIBRARY} ${Xmu_LIBRARY})
	SET(FREEGLUT_LIBRARY ${FREEGLUT_LIBRARIES})
ENDIF(FREEGLUT_FOUND)

MARK_AS_ADVANCED(FREEGLUT_INCLUDE_DIR FREEGLUT_LIBRARY Xi_LIBRARY Xmu_LIBRARY)
