
# GLAROT-3D - Geometric LAndmark relations ROTation-invariant 3D
#### Copyright (C) 2017 Dario Lodi Rizzini.


OVERVIEW
-------------------------------------------------

Library glarot3d implements a geometric signature to select 
loop closure candidates. 
It has been kept to a minimal design. 

If you use this library, please cite the following paper: 

D. Lodi Rizzini. 
Place Recognition of 3D Landmarks based on Geometric Relations. 
IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), sept. 2017. 

or the most relevant associated publications by visiting: 
http://rimlab.ce.unipr.it/


DEPENDENCIES
-------------------------------------------------

The software depends on the following external libraries

- Boost >= 1.36 (submodule lexical_cast)
- Eigen 3.0 

The library also requires the third party library mcqd
developed by Janez Konc (see http://www.sicmm.org/konc/), 
which has been included in folder 3rdparty.


HOW TO COMPILE
-------------------------------------------------

Let ${glarot3d_ROOT} be the install directory of your local copy 
of library glarot3d. 
The following standard commands are required to compile it:

-  cd ${glarot3d_ROOT}
-  mkdir build
-  cd build
-  cmake ..
-  make

You can also install the library into a system directory. 
To change the install directory you must set cmake environment
variable ${CMAKE_INSTALL_PREFIX} (e.g. using command "ccmake .."
before calling "cmake .."). 
Its default value on UNIX-like/Linux systems is "/usr/local".
After compiling library glarot3d, run the command:

-  sudo make install

The command "sudo" is required only if ${CMAKE_INSTALL_PREFIX} 
is a system diretory managed by administrator user root.
Such command copies:
- header files of ${glarot3d_ROOT}/include/glarot3d to
   ${CMAKE_INSTALL_PREFIX}/include/glarot3d/
- library files ${glarot3d_ROOT}/lib/libglarot3d.a to
   ${CMAKE_INSTALL_PREFIX}/lib/
- cmake script ${glarot3d_ROOT}/cmake_modules/glarot3dConfig.cmake to
   ${CMAKE_INSTALL_PREFIX}/share/glarot3d/


HOW TO USE LIBRARY glarot3d IN YOUR PROJECT
-------------------------------------------------

If library glarot3d has been installed in system directory "/usr/local",
then it is straighforward to use it in your projects.
You needs to add the following lines to your project as in this example:


> CMAKE_MINIMUM_REQUIRED(VERSION 2.8)  
> PROJECT(foobar)  
> 
> find_package(glarot3d REQUIRED)  
> message(STATUS "glarot3d_FOUND ${glarot3d_FOUND}")  
> message(STATUS "glarot3d_INCLUDE_DIRS ${glarot3d_INCLUDE_DIRS}")  
> message(STATUS "glarot3d_LIBRARY_DIRS ${glarot3d_LIBRARY_DIRS}")  
> message(STATUS "glarot3d_LIBRARIES ${glarot3d_LIBRARIES}")  
>  
> if(${glarot3d_FOUND})   
>   include_directories(${glarot3d_INCLUDE_DIRS})  
>  link_directories(${glarot3d_LIBRARY_DIRS})  
> endif()  
> 
> add_executable(foobar foobar.cpp)  
> target_link_libraries(foobar ${glarot3d_LIBRARIES})  

The above example uses the variables defined in glarot3dConfig.cmake:

-  glarot3d_FOUND - system has glarot3d module
-  glarot3d_INCLUDE_DIRS - the glarot3d include directories
-  glarot3d_LIBRARY_DIRS - the glarot3d library directories
-  glarot3d_LIBRARIES - link these to use glarot3d


