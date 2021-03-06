# find_package(Qt5Core REQUIRED)
# find_package(Qt5Gui REQUIRED)
SET(REQUIRED_HEADERS lala.h)
SET(REQUIRED_LIBS_NAMES libvtkm_rendering.so)
MESSAGE("-- Using FIND_PACKAGE(VTKm ...) ...")
find_package(VTKm 
PATHS ${VTKM_LIBRARY_DIRS}
  OPTIONAL_COMPONENTS Serial OpenGL Rendering GLUT
  )

if(VTKm_OpenGL_FOUND AND VTKm_Rendering_FOUND AND VTKm_GLUT_FOUND)
#    SET(TPL_VTKM_INCLUDE_DIRS ${VTKm_INCLUDE_DIRS}  ${Qt5Gui_INCLUDE_DIRS} CACHE PATH "...")
    SET(TPL_VTKM_INCLUDE_DIRS ${VTKm_INCLUDE_DIRS} CACHE PATH "...")
    SET(TPL_VTKM_LIBRARIES ${VTKm_LIBRARIES} CACHE FILEPATH "...")
    SET(TPL_VTKM_LIBRARY_DIRS ${VTKm_LIBRARY_DIRS} CACHE PATH "...")
endif()

# Third, call TRIBITS_TPL_FIND_INCLUDE_DIRS_AND_LIBRARIES()
TRIBITS_TPL_FIND_INCLUDE_DIRS_AND_LIBRARIES( VTKM
  REQUIRED_HEADERS ${REQUIRED_HEADERS}
  MUST_FIND_ALL_HEADERS
  REQUIRED_LIBS_NAMES ${REQUIRED_LIBS_NAMES}
  )