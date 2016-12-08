#!/bin/sh
##---------------------------------------------------------------------------##
## CMAKE FOR X86_64
##---------------------------------------------------------------------------##

# CLEANUP
rm -rf CMakeCache.txt
rm -rf CMakeFiles

# SOURCE AND INSTALL
SOURCE=/home/mkim/my-profugus
INSTALL=/usr/local/Profugus

# BUILD OPTIONS
BUILD="DEBUG"
MPI="OFF"

# TPL PATHS
HDF5_PATH="/usr/"
MPI_PATH="/opt/openmpi/gcc/current"

##---------------------------------------------------------------------------##

cmake \
-G "CodeBlocks - Unix Makefiles" \
-DCMAKE_BUILD_TYPE:STRING="$BUILD" \
-DTPL_ENABLE_MPI:BOOL=$MPI \
-DCMAKE_INSTALL_PREFIX:PATH=$INSTALL \
\
-DMPI_BASE_DIR:PATH=$MPI_PATH \
\
-DTPL_ENABLE_VTKM:BOOL="ON"\
-DVTKM_INCLUDE_DIRS:PATH=/home/mkim/local/include/ \
-DVTKM_LIBRARY_DIRS:PATH=/home/mkim/local/lib/ \
\
-DTPL_ENABLE_HDF5:BOOL="ON" \
-DHDF5_INCLUDE_DIRS:PATH=/usr/include/hdf5/serial \
-DHDF5_LIBRARY_DIRS:PATH=/usr/lib/x86_64-linux-gnu/hdf5/serial \
\
-DBLAS_LIBRARY_DIRS:PATH=/usr/lib/libblas \
-DLAPACK_LIBRARY_DIRS:PATH=/usr/lib \
-DBLAS_LIBRARY_NAMES:STRING=";blas" \
-DLAPACK_LIBRARY_NAMES:STRING="lapack" \
\
-DProfugus_CONFIGURE_OPTIONS_FILE:FILEPATH="${SOURCE}/install/base.cmake" \
-DProfugus_ASSERT_MISSING_PACKAGES:BOOL=OFF \
${SOURCE}

##---------------------------------------------------------------------------##
## end of cmake_x86_64.sh
##---------------------------------------------------------------------------##
