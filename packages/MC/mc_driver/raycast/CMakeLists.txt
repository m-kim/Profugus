##=============================================================================
##
##  Copyright (c) Kitware, Inc.
##  All rights reserved.
##  See LICENSE.txt for details.
##
##  This software is distributed WITHOUT ANY WARRANTY; without even
##  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
##  PURPOSE.  See the above copyright notice for more information.
##
##  Copyright 2015 Sandia Corporation.
##  Copyright 2015 UT-Battelle, LLC.
##  Copyright 2015 Los Alamos National Security.
##
##  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
##  the U.S. Government retains certain rights in this software.
##  Under the terms of Contract DE-AC52-06NA25396 with Los Alamos National
##  Laboratory (LANL), the U.S. Government retains certain rights in
##  this software.
##
##=============================================================================
cmake_minimum_required(VERSION 3.3)
project(csg_rendering)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)


#Find the VTK-m package
find_package(VTKm REQUIRED
PATHS "C:\\Program Files\\VTKm\\lib"
  OPTIONAL_COMPONENTS Serial OpenGL Rendering GLUT
  )

if(VTKm_OpenGL_FOUND AND VTKm_Rendering_FOUND AND VTKm_GLUT_FOUND)
  add_executable(parametric_rendering Camera.h Ray.h TriangleIntersector.h RayTracer.h Rendering.cxx MapperRayTracer.cxx)
  target_include_directories(parametric_rendering PRIVATE 	${Qt5Gui_INCLUDE_DIRS} ${VTKm_INCLUDE_DIRS})
  target_link_libraries(parametric_rendering  ${Qt5Gui_LIBRARIES} ${Qt5Core_LIBRARIES} ${VTKm_LIBRARIES})
  target_compile_options(parametric_rendering PRIVATE ${VTKm_COMPILE_OPTIONS})
endif()
