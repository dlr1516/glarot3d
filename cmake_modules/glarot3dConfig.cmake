# - Try to find Library glarot3d
# Once done, this will define
#
#  glarot3d_FOUND - system has glarot3d module
#  glarot3d_INCLUDE_DIRS - the glarot3d include directories
#  glarot3d_LIBRARY_DIRS - the glarot3d library directories
#  glarot3d_LIBRARIES - link these to use glarot3d


# Uses  directory to search mrf_segmentation directory!
set(glarot3d_PREFIX_DIR /usr/local)
message(STATUS "Searching glarot3d in directory ${glarot3d_PREFIX_DIR}." )

# Searches include directory /usr/local/include/glarot3d
find_path(glarot3d_INCLUDE_DIR glarot3d ${glarot3d_PREFIX_DIR}/include)
message(STATUS "    glarot3d_INCLUDE_DIR ${glarot3d_INCLUDE_DIR}." )
set(glarot3d_INCLUDE_DIRS ${glarot3d_INCLUDE_DIR})
  
# Searches library librimagraph.a in /usr/local/lib
find_path(glarot3d_LIBRARY_DIR librimagraph.a ${glarot3d_PREFIX_DIR}/lib)
message(STATUS "    glarot3d_LIBRARY_DIR ${glarot3d_LIBRARY_DIR}." )
set(glarot3d_LIBRARY_DIRS ${glarot3d_PREFIX_DIR}/lib)

# Sets the names of library components (actually A name and A component)
find_library(glarot3d_LIBRARY glarot3d ${glarot3d_LIBRARY_DIRS})
message(STATUS "    glarot3d_LIBRARY ${glarot3d_LIBRARY}." )
set(glarot3d_LIBRARIES ${glarot3d_LIBRARY})

if(("${glarot3d_INCLUDE_DIR}" STREQUAL "glarot3d_INCLUDE_DIR-NOTFOUND") OR
   ("${glarot3d_LIBRARY_DIRS}" STREQUAL "glarot3d_LIBRARY_DIRS-NOTFOUND") OR
   ("${glarot3d_LIBRARY}" STREQUAL "glarot3d_LIBRARY-NOTFOUND")
  )
  message(STATUS "Library glarot3d NOT found")
  unset(glarot3d_FOUND)
  unset(glarot3d_INCLUDE_DIR)
  unset(glarot3d_LIBRARY_DIR)
  unset(glarot3d_LIBRARY)
  unset(glarot3d_LIBRARIES)
endif()

mark_as_advanced(glarot3d_INCLUDE_DIRS glarot3d_LIBRARY_DIRS glarot3d_LIBRARIES)
