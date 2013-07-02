# Copyright 2012, Olivier Stasse, JRL, CNRS/AIST
# Requires at least CMake 2.6 to configure the package.
cmake_minimum_required(VERSION 2.8)
project(mpc-walkgen)
enable_testing()
find_package(qibuild)

# Handle OS specificities
include(CheckIncludeFiles)
check_include_files("sys/time.h" SYS_TIME_H)
if(SYS_TIME_H)
  add_definitions("-DHAVE_SYS_TIME_H")
endif()

# make all symbols hidden with gcc.
if(UNIX)
  add_definitions("-fvisibility=hidden")
endif()

qi_add_optional_package(LSSOL)
qi_add_optional_package(QPOASES)

if(NOT WITH_LSSOL AND NOT WITH_QPOASES)
  message(FATAL_ERROR "No QP solver found.  Possible solvers: LSSOL and qpOASES")
endif()

if(WITH_QPOASES)
  # There are two versions of qpOASES, that change the way the inclusion
  # has to be done.
  if(NOT QPOASES_VERSION VERSION_LESS 3.0)
    add_definitions("-DUSE_QPOASES_3_0")
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
      set(_QPSOLVERTYPE_ENUM "${_QPSOLVERTYPE_ENUM},\n")
    endif()
    # add the enum entry
    set(_QPSOLVERTYPE_ENUM "${_QPSOLVERTYPE_ENUM}\t\tQPSOLVERTYPE_${solver}")
    set(_QPSOLVERTYPE_ENUM "${_QPSOLVERTYPE_ENUM} = ${value}")
  endif()
endmacro()
addSolverToEnum("LSSOL" "0")
addSolverToEnum("QPOASES" "1")
set(_inc mpc-walkgen/common/qp-solver-type.h)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${_inc}.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${_inc}" @ONLY)
set_source_files_properties("${CMAKE_CURRENT_BINARY_DIR}/${_inc}" GENERATED)

qi_install_header("${CMAKE_CURRENT_BINARY_DIR}/${_inc}"
  SUBFOLDER mpc-walkgen/common)

set(_HEADERS_GENERATED
  ${CMAKE_CURRENT_BINARY_DIR}/${_inc})

set(_HEADERS_NOT_GENERATED
  mpc-walkgen/common/api.h
  mpc-walkgen/common/sharedpgtypes.h
  mpc-walkgen/humanoid/walkgen-abstract.h
  mpc-walkgen/humanoid/sharedpgtypes.h
  mpc-walkgen/zebulon/walkgen-abstract.h
  mpc-walkgen/zebulon/sharedpgtypes.h)

qi_install_header(${_HEADERS_NOT_GENERATED} KEEP_RELATIVE_PATHS)

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
  list(APPEND mpc-walkgen_SRC src/common/qp-solvers/lssol-solver.cpp)
endif()

if(WITH_QPOASES)
  list(APPEND mpc-walkgen_SRC src/common/qp-solvers/qpoases-solver.cpp)
endif()

include_directories("${CMAKE_CURRENT_BINARY_DIR}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

qi_create_lib(mpc-walkgen SHARED
  ${mpc-walkgen_SRC}
  ${_HEADERS_GENERATED}
  ${_HEADERS_NOT_GENERATED})
qi_use_lib(mpc-walkgen eigen3 QI)
if(WITH_LSSOL)
  qi_use_lib(mpc-walkgen LSSOL)
endif()

if(WITH_QPOASES)
  qi_use_lib(mpc-walkgen QPOASES)
endif()
qi_stage_lib(mpc-walkgen)

add_subdirectory(test)

add_subdirectory(new)
