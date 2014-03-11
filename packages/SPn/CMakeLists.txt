##---------------------------------------------------------------------------##
## SPn/CMakeLists.txt
## Thomas M. Evans
## Monday June 20 15:40:52 2011
##---------------------------------------------------------------------------##
## Copyright (C) 2011 Oak Ridge National Laboratory, UT-Battelle, LLC.
##---------------------------------------------------------------------------##
## TriBITS package file for SPn package
##---------------------------------------------------------------------------##

INCLUDE(TribitsPackageMacros)
INCLUDE(TribitsLibraryMacros)
INCLUDE(TribitsAddOptionAndDefine)
INCLUDE(AppendSet)
INCLUDE(ProfugusAddCFlags)

# Add Utils cmake to path
APPEND_SET(CMAKE_MODULE_PATH ${Utils_SOURCE_DIR}/cmake)

##---------------------------------------------------------------------------##
## A) Define the package
##---------------------------------------------------------------------------##

TRIBITS_PACKAGE(SPn)

##---------------------------------------------------------------------------##
## B) Set up package-specific options
##---------------------------------------------------------------------------##

## ML OPTIONS

## Turn on ML if that package is enabled
IF (${PROJECT_NAME}_ENABLE_ML)
  SET(USE_ML 1)
ENDIF()

# Setup M4 for FORTRAN processing
APPEND_SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Setup debug option
TRIBITS_ADD_DEBUG_OPTION()

# to allow includes like #include "comm/Comm.h"
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

# to allow includes like #include "Utils/config.h"
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/..)

IF(CMAKE_COMPILER_IS_GNUCXX AND NOT WIN32)
  PROFUGUS_ADD_CXX_FLAGS(
    -Wno-sign-compare
    -Wno-unused-but-set-variable
    -Wno-unused-variable)
ENDIF()

#
# Resolve min/max conflict on Windows
#
IF( CMAKE_SYSTEM_NAME MATCHES "Windows" )
   ADD_DEFINITIONS(-DNOMINMAX)
ENDIF( CMAKE_SYSTEM_NAME MATCHES "Windows" )

##---------------------------------------------------------------------------##
# C) Process the subpackages
##---------------------------------------------------------------------------##

TRIBITS_CONFIGURE_FILE(config.h)

##---------------------------------------------------------------------------##
## SOURCES AND HEADERS

SET(SOURCES "")
SET(HEADERS "")

# mesh sources

FILE(GLOB MESH_HEADERS mesh/*.hh)
SET(MESH_SOURCES
  mesh/LG_Indexer.cc
  mesh/Mesh.cc
  mesh/Partitioner.cc
  )
LIST(APPEND HEADERS ${MESH_HEADERS})
LIST(APPEND SOURCES ${MESH_SOURCES})

# solvers sources

FILE(GLOB SOLVERS_HEADERS solvers/*.hh)
SET(SOLVERS_SOURCES
  solvers/Arnoldi.cc
  solvers/Davidson_Eigensolver.cc
  solvers/Decomposition.cc
  solvers/EigenvalueSolverBuilder.cc
  solvers/InverseOperator.cc
  solvers/LinearSolverBuilder.cc
  solvers/PowerIteration.pt.cc
  solvers/PreconditionerBuilder.cc
  solvers/RayleighQuotient.cc
  solvers/Richardson.pt.cc
  solvers/ShiftedInverseOperator.cc
  solvers/ShiftedOperator.cc
  solvers/StratimikosSolver.cc
  )
LIST(APPEND HEADERS ${SOLVERS_HEADERS})
LIST(APPEND SOURCES ${SOLVERS_SOURCES})

# spn sources

FILE(GLOB SPN_HEADERS spn/*.hh)
SET(SPN_SOURCES
  spn/Dimensions.cc
  spn/Eigenvalue_Solver.cc
  spn/Energy_Multigrid.cc
  spn/Energy_Prolongation.cc
  spn/Energy_Restriction.cc
  spn/FV_Bnd_Indexer.cc
  spn/FV_Gather.cc
  spn/Fixed_Source_Solver.cc
  spn/Isotropic_Source.cc
  spn/Linear_System.cc
  spn/Linear_System_FV.cc
  spn/Moment_Coefficients.cc
  spn/SDM_Face_Field.cc
  spn/Solver_Base.cc
  )
LIST(APPEND HEADERS ${SPN_HEADERS})
LIST(APPEND SOURCES ${SPN_SOURCES})

LIST(APPEND HEADERS ${CMAKE_CURRENT_BINARY_DIR}/config.h)

##---------------------------------------------------------------------------##
## LIBRARY AND INSTALL TARGETS

TRIBITS_ADD_LIBRARY(
  SPn
  NOINSTALLHEADERS ${HEADERS}
  SOURCES ${SOURCES})

##---------------------------------------------------------------------------##
# Add tests to this package

TRIBITS_ADD_TEST_DIRECTORIES(
  mesh/test
  solvers/test
  spn/test)

##---------------------------------------------------------------------------##
## FINISH SETUP

# Exclude files for source package.
TRIBITS_EXCLUDE_AUTOTOOLS_FILES()

##---------------------------------------------------------------------------##
## D) Do standard postprocessing
##---------------------------------------------------------------------------##

TRIBITS_PACKAGE_POSTPROCESS()

##---------------------------------------------------------------------------##
##                      end of Utils/CMakeLists.txt
##---------------------------------------------------------------------------##