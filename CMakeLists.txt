# Copyright 2012, Olivier Stasse, JRL, CNRS/AIST
# Requires at least CMake 2.6 to configure the package.
cmake_minimum_required(VERSION 2.8)


# Define properties of the project
SET(PROJECT_NAME mpc-walkgen)
SET(PROJECT_DESCRIPTION "MPC Walking pattern generator for Humanoids")
SET(PROJECT_URL "https://gforge.inria.fr/projects/mpc-walkgen")
SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")
SET(CXX_DISABLE_WERROR True)

find_package(jrl-cmake REQUIRED
    HINTS cmake)
INCLUDE(${jrl-cmake_DIR}/base.cmake)
INCLUDE(${jrl-cmake_DIR}/cpack.cmake)
INCLUDE(cmakeutils.cmake)
# custom cmake modules to find dependencies
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules")

# Initialize generic properties of the project
SETUP_PROJECT()

# Handle OS specificities
INCLUDE(CheckIncludeFiles)
CHECK_INCLUDE_FILES("sys/time.h" SYS_TIME_H)
IF(SYS_TIME_H)
  ADD_DEFINITIONS("-DHAVE_SYS_TIME_H")
ENDIF(SYS_TIME_H)
# make all symbols hidden with gcc.
if(UNIX)
  ADD_DEFINITIONS("-fvisibility=hidden")
endif()

find_package(qibuild QUIET)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Optional dependencies: use an external QP solver library.
find_optional_package(
  LSSOL "build mpc-walkgen with support for LSSOL QP solver ")
find_optional_package(
  QPOASES "build mpc-walkgen with support for qpOASES QP solver")

if(NOT WITH_LSSOL AND NOT WITH_QPOASES)
  message(FATAL_ERROR
    "No QP solver found.  Possible solvers: LSSOL and qpOASES")
endif()

if(WITH_QPOASES)
  # There are two versions of qpOASES, that change the way the inclusion
  # has to be done.
  if(NOT QPOASES_VERSION VERSION_LESS 3.0)
    ADD_DEFINITIONS("-DUSE_QPOASES_3_0")
  endif()
endif()

# addSolverToHeader. If the solver support is enabled, add it to the variables
# used for header generation:
#  - set the MPC_WALKGEN_WITH_${solver} variable
#  - add an enum entry to the _QPSOLVERTYPE_ENUM variable
#
# arguments:
#  - solver:     name of the solver
#  - value:      value associated to the enum entry
macro(addSolverToEnum solver value)
  if(WITH_${solver})
      # Let mpc-walkgen and its users know that the solver support is built-in.
      set(MPC_WALKGEN_WITH_${solver} ON)
    if(_QPSOLVERTYPE_ENUM)
      # add a line to the enum, except for the very first line
      SET(_QPSOLVERTYPE_ENUM "${_QPSOLVERTYPE_ENUM},\n")
    endif()
    # add the enum entry
    SET(_QPSOLVERTYPE_ENUM "${_QPSOLVERTYPE_ENUM}\t\tQPSOLVERTYPE_${solver}")
    SET(_QPSOLVERTYPE_ENUM "${_QPSOLVERTYPE_ENUM} = ${value}")
  endif()
endmacro()
addSolverToEnum("LSSOL" "0")
addSolverToEnum("QPOASES" "1")
CONFIG_FILES(include/mpc-walkgen/common/qp-solver-type.h)

# Define the set of headers to be installed
SET(${PROJECT_NAME}_HEADERS
  include/mpc-walkgen/common/api.h
  ${CMAKE_BINARY_DIR}/include/mpc-walkgen/common/qp-solver-type.h
  include/mpc-walkgen/common/sharedpgtypes.h

  include/mpc-walkgen/humanoid/walkgen-abstract.h
  include/mpc-walkgen/humanoid/sharedpgtypes.h

  include/mpc-walkgen/zebulon/walkgen-abstract.h
  include/mpc-walkgen/zebulon/sharedpgtypes.h
)

SET(mpc-walkgen_SRC
  src/common/types.cpp
  src/common/types.h
  src/common/gettimeofday.cpp
  src/common/gettimeofday.h
  src/common/interpolation.cpp
  src/common/interpolation.h
  src/common/mpc-debug.cpp
  src/common/mpc-debug.h
  src/common/qp-matrix.cpp
  src/common/qp-matrix.h
  src/common/qp-solver.cpp
  src/common/qp-solver.h
  src/common/qp-solvers/lssol-solver.h
  src/common/qp-solvers/qpoases-solver.h
  src/common/qp-vector.cpp
  src/common/qp-vector.h
  src/common/tools.cpp
  src/common/tools.h
  src/common/tools-inl.h
  src/common/sharedpgtypes.cpp

  src/humanoid/orientations-preview.cpp
  src/humanoid/orientations-preview.h
  src/humanoid/qp-generator.cpp
  src/humanoid/qp-generator.h
  src/humanoid/qp-preview.cpp
  src/humanoid/qp-preview.h
  src/humanoid/rigid-bodies/com-body.cpp
  src/humanoid/rigid-bodies/com-body.h
  src/humanoid/rigid-bodies/foot-body.cpp
  src/humanoid/rigid-bodies/foot-body.h
  src/humanoid/rigid-body.cpp
  src/humanoid/rigid-body.h
  src/humanoid/rigid-body-system.cpp
  src/humanoid/rigid-body-system.h
  src/humanoid/state-solver.cpp
  src/humanoid/state-solver.h
  src/humanoid/types.cpp
  src/humanoid/types.h
  src/humanoid/walkgen.cpp
  src/humanoid/walkgen.h
  src/humanoid/sharedpgtypes.cpp
  src/humanoid/walkgen-abstract.cpp

  src/zebulon/qp-generator.cpp
  src/zebulon/qp-generator.h
  src/zebulon/qp-generator-orientation.cpp
  src/zebulon/qp-generator-orientation.h
  src/zebulon/rigid-bodies/com-body.cpp
  src/zebulon/rigid-bodies/com-body.h
  src/zebulon/rigid-bodies/base-body.cpp
  src/zebulon/rigid-bodies/base-body.h
  src/zebulon/rigid-body.cpp
  src/zebulon/rigid-body.h
  src/zebulon/rigid-body-system.cpp
  src/zebulon/rigid-body-system.h
  src/zebulon/types.cpp
  src/zebulon/types.h
  src/zebulon/walkgen.cpp
  src/zebulon/walkgen.h
  src/zebulon/sharedpgtypes.cpp
  src/zebulon/walkgen-abstract.cpp
)

if(WITH_LSSOL)
  list(APPEND mpc-walkgen_SRC
    src/common/qp-solvers/lssol-solver.cpp
  )
endif()

if(WITH_QPOASES)
  list(APPEND mpc-walkgen_SRC
    src/common/qp-solvers/qpoases-solver.cpp
  )
endif()

INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/include)

ADD_LIBRARY(mpc-walkgen SHARED ${mpc-walkgen_SRC} ${${PROJECT_NAME}_HEADERS})
SET_TARGET_PROPERTIES(mpc-walkgen PROPERTIES SOVERSION ${PROJECT_VERSION})
INSTALL(TARGETS mpc-walkgen DESTINATION lib)

PKG_CONFIG_APPEND_CFLAGS("-I${EIGEN3_INCLUDE_DIR}")
if(WITH_LSSOL)
  PKG_CONFIG_USE_DEPENDENCY(mpc-walkgen lssol)
endif()

if(WITH_QPOASES)
  PKG_CONFIG_USE_DEPENDENCY(mpc-walkgen qpoases)
endif()

# Define subdirectories to explore for cmake
ADD_SUBDIRECTORY(unittest)

# Generate dependency to jrl-walkgen in pc file
PKG_CONFIG_APPEND_LIBS("mpc-walkgen")

# Ask to generate the final steps.
SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()

