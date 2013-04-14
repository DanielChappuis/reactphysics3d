add_definitions( -DFREEGLUT_EXPORTS -DFREEGLUT_STATIC -D_CRT_SECURE_NO_WARNINGS )

if(APPLE)
	include_directories( /usr/X11/include )
endif(APPLE)

if(UNIX)
	add_definitions( -D__unix__ -DHAVE_FCNTL_H -DHAVE_GETTIMEOFDAY )
endif(UNIX)

file (
  GLOB_RECURSE
  FREEGLUT_SOURCES
  ./*
)

include_directories (
	${OPENGL_INCLUDE_DIR}
	.
)

add_library(freeglut_static
	${FREEGLUT_SOURCES}
)
