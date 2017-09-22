cmake_minimum_required(VERSION 2.8)
project(glarot3d)
add_definitions(-std=c++0x) 
set(CMAKE_BUILD_TYPE RelWithDebug)

SET(glarot3d_RUNTIME_OUTPUT_DIRECTORY ${glarot3d_SOURCE_DIR}/bin CACHE PATH "Target for the binaries")
SET(glarot3d_LIBRARY_OUTPUT_DIRECTORY ${glarot3d_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${glarot3d_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${glarot3d_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${glarot3d_RUNTIME_OUTPUT_DIRECTORY})
LIST(APPEND CMAKE_MODULE_PATH ${glarot3d_SOURCE_DIR}/cmake_modules)

# Library Boost
find_package(Boost REQUIRED COMPONENTS iostreams system filesystem)

# Library Eigen3
find_package(Eigen3 REQUIRED)
if (${EIGEN3_FOUND})
  include_directories(${EIGEN3_INCLUDE_DIR})
  message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
else(${EIGEN3_FOUND})
  message(WARNING "Cannot find Eigen3 Library")
endif(${EIGEN3_FOUND})

# Local include
include_directories(include)

add_library(glarot3d 
  src/CorrespondenceGraph.cpp
  src/CubeSymmetry.cpp 
  src/Glarot3d.cpp 
  src/ParamMap.cpp
  src/Profiler.cpp
  src/utils.cpp
)

add_executable(processOrbSlamMap src/processOrbSlamMap.cpp)
target_link_libraries(processOrbSlamMap ${Boost_LIBRARIES} glarot3d)

add_executable(computeStats src/computeStats.cpp)
target_link_libraries(computeStats ${Boost_LIBRARIES} glarot3d)



# Option "make install": copy binaries
INSTALL(TARGETS glarot3d 
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

# Option "make install": copy headers
FILE(GLOB glarot3d_includes "${CMAKE_CURRENT_SOURCE_DIR}/include/glarot3d/*.h")
FILE(GLOB thirdparty_includes "${CMAKE_CURRENT_SOURCE_DIR}/include/glarot3d/3rdparty/*.h")
INSTALL(FILES ${glarot3d_includes} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/glarot3d)
INSTALL(FILES ${thirdparty_includes} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/glarot3d/3rdparty)

# Option "make install": copy cmake script
FILE(GLOB cmake_script "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/glarot3dConfig.cmake")
message(STATUS "cmake_script " ${cmake_script})
INSTALL(FILES ${cmake_script} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/glarot3d/)
