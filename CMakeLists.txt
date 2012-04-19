# Copyright 2012, Olivier Stasse, JRL, CNRS/AIST
# Requires at least CMake 2.6 to configure the package.
cmake_minimum_required(VERSION 2.8)
project(mpc-walkgen)
# Define properties of the project
SET(PROJECT_NAME mpc-walkgen)
SET(PROJECT_DESCRIPTION "MPC Walking pattern generator for Humanoids")
SET(PROJECT_URL "")

find_package(qibuild) # it does not matter if qibuild is not found

# Includes from jrl-cmaketools
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/cpack.cmake)

# custom cmake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")
SET(CXX_DISABLE_WERROR True)

# Initialize generic properties of the project
SETUP_PROJECT()
SEARCH_FOR_BOOST()
SEARCH_FOR_EIGEN()

# Optional dependencies: use a external qp solver library.
ADD_OPTIONAL_DEPENDENCY("lssol >= 0.1.0")
FIND_PACKAGE(QPOASES)

# Crash if no dependency was found.
if(NOT LSSOL_FOUND AND NOT QPOASES_FOUND)
  message(FATAL_ERROR
    "No QP solver found.  Possible solvers: LSSOL and qpOASES")
endif(NOT LSSOL_FOUND AND NOT QPOASES_FOUND)

if(LSSOL_FOUND)
  ADD_DEFINITIONS("-DUSE_LSSOL")
endif(LSSOL_FOUND)

if(QPOASES_FOUND)
  ADD_DEFINITIONS("-DUSE_QPOASES")
  #There seems to be two version of qpoases, that change the way the inclusion
  # has to be done.
  if(NOT QPOASES_VERSION VERSION_LESS 3.0)
    ADD_DEFINITIONS("-DUSE_QPOASES_3_0")
  endif()
endif(QPOASES_FOUND)


# make all symbols hidden with gcc.
if(UNIX)
  ADD_DEFINITIONS("-fvisibility=hidden")
endif()
# Define the set of headers to be installed
SET(${PROJECT_NAME}_HEADERS
  include/mpc-walkgen/api.h
  include/mpc-walkgen/sharedpgtypes.h
  include/mpc-walkgen/walkgen-abstract.h
  ${CMAKE_BINARY_DIR}/include/mpc-walkgen/solvers.h
)

# Define subdirectories to explore for cmake
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(unittest)

# Generate dependency to jrl-walkgen in pc file
PKG_CONFIG_APPEND_LIBS("mpc-walkgen")
PKG_CONFIG_APPEND_CFLAGS(${_Eigen_CFLAGS})

# Ask to generate the final steps.
SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()

