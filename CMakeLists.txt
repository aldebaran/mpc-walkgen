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

# Initialize generic properties of the project
SETUP_PROJECT()

# custom cmake modules to find dependencies
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules")

find_package(qibuild QUIET)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost 1.40
    COMPONENTS filesystem system thread program_options unit_test_framework
    REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# Handle OS specificities
INCLUDE(CheckIncludeFiles)
CHECK_INCLUDE_FILES("sys/time.h" SYS_TIME_H)
IF(SYS_TIME_H)
 ADD_DEFINITIONS("-DHAVE_SYS_TIME_H")
ENDIF(SYS_TIME_H)

# Optional dependencies: use an external QP solver library.
ADD_OPTIONAL_DEPENDENCY("lssol >= 0.1.0")
FIND_PACKAGE(QPOASES)
if(NOT LSSOL_FOUND AND NOT QPOASES_FOUND)
  # Crash if no QP solver was found.
  message(FATAL_ERROR
    "No QP solver found.  Possible solvers: LSSOL and qpOASES")
endif(NOT LSSOL_FOUND AND NOT QPOASES_FOUND)

if(LSSOL_FOUND)
  ADD_DEFINITIONS("-DUSE_LSSOL")
endif(LSSOL_FOUND)

if(QPOASES_FOUND)
  ADD_DEFINITIONS("-DUSE_QPOASES")
  #There are two versions of qpOASES, that change the way the inclusion
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
  include/mpc-walkgen/qp-solver-type.h
  include/mpc-walkgen/sharedpgtypes-common.h

  include/mpc-walkgen/walkgen-abstract-humanoid.h
  include/mpc-walkgen/sharedpgtypes-humanoid.h

  include/mpc-walkgen/walkgen-abstract-zebulon.h
  include/mpc-walkgen/sharedpgtypes-zebulon.h
)

SET(mpc-walkgen_SRC
  src/common/common-types.cpp
  src/common/common-types.h
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
  src/sharedpgtypes-common.cpp

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
  src/sharedpgtypes-humanoid.cpp
  src/walkgen-abstract-humanoid.cpp

  src/zebulon/qp-generator.cpp
  src/zebulon/qp-generator.h
  src/zebulon/rigid-bodies/com-body.cpp
  src/zebulon/rigid-bodies/com-body.h
  src/zebulon/rigid-bodies/base-body.cpp
  src/zebulon/rigid-bodies/base-body.h
  src/zebulon/rigid-body.cpp
  src/zebulon/rigid-body.h
  src/zebulon/rigid-body-system.cpp
  src/zebulon/rigid-body-system.h
  src/zebulon/types.h
  src/zebulon/walkgen.cpp
  src/zebulon/walkgen.h
  src/sharedpgtypes-zebulon.cpp
  src/walkgen-abstract-zebulon.cpp
)

if(LSSOL_FOUND)
  list(APPEND mpc-walkgen_SRC
    src/common/qp-solvers/lssol-solver.cpp
  )
endif(LSSOL_FOUND)

if(QPOASES_FOUND)
  list(APPEND mpc-walkgen_SRC
    src/common/qp-solvers/qpoases-solver.cpp
  )
endif(QPOASES_FOUND)

ADD_LIBRARY(mpc-walkgen SHARED ${mpc-walkgen_SRC} ${${PROJECT_NAME}_HEADERS})
SET_TARGET_PROPERTIES(mpc-walkgen PROPERTIES SOVERSION ${PROJECT_VERSION})
INSTALL(TARGETS mpc-walkgen DESTINATION lib)

if(LSSOL_FOUND)
  PKG_CONFIG_USE_DEPENDENCY(mpc-walkgen lssol)
endif(LSSOL_FOUND)

if(QPOASES_FOUND)
  PKG_CONFIG_USE_DEPENDENCY(mpc-walkgen qpoases)
endif(QPOASES_FOUND)

# Define subdirectories to explore for cmake
ADD_SUBDIRECTORY(unittest)

# Generate dependency to jrl-walkgen in pc file
PKG_CONFIG_APPEND_LIBS("mpc-walkgen")

# Ask to generate the final steps.
SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()

