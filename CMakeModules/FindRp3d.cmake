# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#[=======================================================================[.rst:
FindRp3d
-------

Finds the React Physics 3D library.

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported targets, if found:

``Rp3d::Rp3d``
  The React Physics 3D library

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``Rp3d_FOUND``
  True if the system has the React Physics 3D library.
``Rp3d_VERSION``
  The version of the React Physics 3D library which was found.
``Rp3d_INCLUDE_DIRS``
  Include directories needed to use React Physics 3D.
``Rp3d_LIBRARIES``
  Libraries needed to link to React Physics 3D.

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``Rp3d_INCLUDE_DIR``
  The directory containing ``reactphysics3d.h``.
``Rp3d_LIBRARY``
  The path to the React Physics 3D library.

#]=======================================================================]

find_package(PkgConfig)
pkg_check_modules(PC_Rp3d QUIET Rp3d)

find_path(Rp3d_INCLUDE_DIR
  NAMES reactphysics3d.h
  PATHS ${PC_Rp3d_INCLUDE_DIRS}
  PATH_SUFFIXES reactphysics3d
  HINTS "/opt/rp3d/include"
)
find_library(Rp3d_LIBRARY
  NAMES reactphysics3d
  PATHS ${PC_Rp3d_LIBRARY_DIRS}
  HINTS "/opt/rp3d/lib"
)


#set(Foo_VERSION ${PC_Foo_VERSION})
#



include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Rp3d
  FOUND_VAR Rp3d_FOUND
  REQUIRED_VARS
    Rp3d_LIBRARY
    Rp3d_INCLUDE_DIR
  VERSION_VAR Rp3d_VERSION
)


if (${Rp3d_FOUND})
	file(STRINGS "${Rp3d_INCLUDE_DIR}/configuration.h" Rp3d_VERSION_LINE REGEX "RP3D_VERSION = std::string\\(\"([0-9.]+)\"\\)")
	string(REGEX MATCH "RP3D_VERSION = std::string\\(\"([0-9.]+)\"\\)" _ ${Rp3d_VERSION_LINE})
	set(Rp3d_VERSION ${CMAKE_MATCH_1})
endif()


if(Rp3d_FOUND)
  set(Rp3d_LIBRARIES ${Rp3d_LIBRARY})
  set(Rp3d_INCLUDE_DIRS ${Rp3d_INCLUDE_DIR})
  set(Rp3d_DEFINITIONS ${PC_Rp3d_CFLAGS_OTHER})
endif()


if(Rp3d_FOUND AND NOT TARGET Rp3d::Rp3d)
  add_library(Rp3d::Rp3d UNKNOWN IMPORTED)
  set_target_properties(Rp3d::Rp3d PROPERTIES
    IMPORTED_LOCATION "${Rp3d_LIBRARY}"
    INTERFACE_COMPILE_OPTIONS "${PC_Rp3d_CFLAGS_OTHER}"
    INTERFACE_INCLUDE_DIRECTORIES "${Rp3d_INCLUDE_DIR}"
  )
endif()

mark_as_advanced(
  Rp3d_INCLUDE_DIR
  Rp3d_LIBRARY
)
