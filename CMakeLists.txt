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

# Handle OS specificities 
INCLUDE(CheckIncludeFiles)
CHECK_INCLUDE_FILES("sys/time.h" SYS_TIME_H)
IF(SYS_TIME_H)
 ADD_DEFINITIONS("-DHAVE_SYS_TIME_H")
ENDIF(SYS_TIME_H)

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

# addSolver:  create the list of solvers available that will be used
#             to create a enum list in C++
# solver:     the name of the enum.
# solverList: the enum list
macro(addSolver solver solverList)
  if(NOT ${solverList})
    SET(${solverList} "\t\t${solver}")
  else(NOT solverList)
    SET(${solverList} "${${solverList}},\n\t\t${solver}")
  endif(NOT ${solverList})
endmacro(addSolver)

set (_SOLVER_LIST_ "")
if(LSSOL_FOUND)
  addSolver("LSSOL" _SOLVER_LIST_)
endif(LSSOL_FOUND)

if(QPOASES_FOUND)
  addSolver("QPOASES" _SOLVER_LIST_)
endif(QPOASES_FOUND)
CONFIG_FILES(include/mpc-walkgen/solvers.h)
# Define the set of headers to be installed
SET(${PROJECT_NAME}_HEADERS
  include/mpc-walkgen/api.h
  include/mpc-walkgen/sharedpgtypes.h
  include/mpc-walkgen/sharedpgtypes-humanoid.h
  include/mpc-walkgen/walkgen-abstract.h
  include/mpc-walkgen/walkgen-abstract-humanoid.h
  ${CMAKE_BINARY_DIR}/include/mpc-walkgen/solvers.h
)

SET(mpc-walkgen_SRC
 src/sharedpgtypes.cpp
 src/sharedpgtypes-humanoid.cpp
 src/walkgen-abstract.cpp
 src/walkgen-abstract-humanoid.cpp

 src/walkgen/convex-hull.cpp
 src/walkgen/orientations-preview.cpp

 src/common/qp-solver.h
 src/common/qp-solvers/lssol-solver.h
 src/common/qp-solvers/qpoases-solver.h
 src/common/qp-solver.cpp
 src/common/qp-matrix.cpp
 src/common/qp-vector.cpp
 src/common/qp-matrix.h
 src/common/qp-vector.h
 src/common/common-types.h
 src/common/tools.h
 src/common/tools-inl.h
 src/common/tools.cpp

 src/walkgen/state-solver.cpp
 src/walkgen/state-solvers/fsm-solver.cpp
 src/walkgen/rigid-body-system.h
 src/walkgen/rigid-body.h
 src/walkgen/rigid-bodies/com-body.h
 src/walkgen/rigid-bodies/foot-body.h
 src/walkgen/rigid-body-system.cpp
 src/walkgen/rigid-body.cpp
 src/walkgen/rigid-bodies/com-body.cpp
 src/walkgen/rigid-bodies/foot-body.cpp

 src/walkgen/qp-generator.cpp
 src/walkgen/qp-preview.cpp
 src/walkgen/interpolation.cpp

 src/walkgen/types.cpp
 src/walkgen/walkgen-humanoid.cpp
 src/walkgen/gettimeofday.cpp
 src/walkgen/mpc-debug.cpp
 src/walkgen/perturbation.cpp
 src/walkgen/orientations-preview.h
 src/walkgen/state-solver.h
 src/walkgen/state-solvers/fsm-solver.h

 src/walkgen/qp-generator.h
 src/walkgen/qp-preview.h
 src/walkgen/interpolation.h

 src/walkgen/types.h
 src/walkgen/enums.h
 src/walkgen/walkgen-humanoid.h
 src/walkgen/mpc-debug.h
 src/walkgen/perturbation.h
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

# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

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
PKG_CONFIG_APPEND_CFLAGS(${_Eigen_CFLAGS})

# Ask to generate the final steps.
SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()

