# Load the file listing the sources required for each unit test.
include(SourcesLib.cmake)

INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src)

# define a macro wrapper to add an unit test.
#  name is the name of the unit test.
#  internal_dep indicates if the internal dep (aka does that depends on mpc-walkgen?)
#  external_dep lists the external dependencies (such as the solver)
macro(addUnitTest name internal_dep external_dep)
  ADD_EXECUTABLE(${name}  ${${name}_SRC})
  # avoid the Debug/Release subdirs in VS, so that we can find the executable
  SET(_output_directory "${CMAKE_CURRENT_BINARY_DIR}")
  SET_TARGET_PROPERTIES("${name}" PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${_output_directory}"
      RUNTIME_OUTPUT_DIRECTORY_RELEASE "${_output_directory}"
      RUNTIME_OUTPUT_DIRECTORY         "${_output_directory}"
  )
  # Now that all configs end up in the same directory, we need to avoid name
  # clashes
  if(MSVC)
    # always postfix debug lib/bin with _d ...
    SET_TARGET_PROPERTIES("${name}" PROPERTIES DEBUG_POSTFIX "_d")
  endif()

  set(_exec_path ${_output_directory}/${name})
  if(MSVC AND "${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    set(_exec_path ${_exec_path}_d)
  endif()
  ADD_TEST(${name} "${_exec_path}")

  foreach(dep ${internal_dep})
    ADD_DEPENDENCIES(${name} ${dep})
    TARGET_LINK_LIBRARIES(${name} ${dep})
  endforeach(dep)

  foreach(dep ${external_dep})
    PKG_CONFIG_USE_DEPENDENCY(${name} ${dep})
  endforeach(dep)
endmacro(addUnitTest)

macro(addBench name internal_dep external_dep)
  ADD_EXECUTABLE(${name}  ${${name}_SRC})
  # avoid the Debug/Release subdirs in VS, so that we can find the executable
  SET(_output_directory "${CMAKE_CURRENT_BINARY_DIR}")
  SET_TARGET_PROPERTIES("${name}" PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${_output_directory}"
      RUNTIME_OUTPUT_DIRECTORY_RELEASE "${_output_directory}"
      RUNTIME_OUTPUT_DIRECTORY         "${_output_directory}"
  )
  # Now that all configs end up in the same directory, we need to avoid name
  # clashes
  if(MSVC)
    # always postfix debug lib/bin with _d ...
    SET_TARGET_PROPERTIES("${name}" PROPERTIES DEBUG_POSTFIX "_d")
  endif()

  set(_exec_path ${_output_directory}/${name})
  if(MSVC AND "${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    set(_exec_path ${_exec_path}_d)
  endif()

  foreach(dep ${internal_dep})
    ADD_DEPENDENCIES(${name} ${dep})
    TARGET_LINK_LIBRARIES(${name} ${dep})
  endforeach(dep)

  foreach(dep ${external_dep})
    PKG_CONFIG_USE_DEPENDENCY(${name} ${dep})
  endforeach(dep)
endmacro(addUnitTest)

## Add static libraries to gather some code used in the unit tests
## and avoid compiling the same files over and over
add_library(solver STATIC ${solver_SRC})
add_library(timer STATIC ${timer_SRC})

## test the rotation of the cholesky matrix
addUnitTest(test-rotation "timer" "")

## Testing the solvers
set(solverAvailable "")

# test the lssol solver
if(WITH_LSSOL)
  list(APPEND solverAvailable "lssol")
  addUnitTest(test-lssol "" "lssol")
endif()


# test the qpoases solver
if(WITH_QPOASES)
  list(APPEND solverAvailable "QPOASES")
  addUnitTest(test-qpoases "" "QPOASES")
endif()


## Testing all the solvers
addUnitTest(bench-qpsolver   "solver;timer" "${solverAvailable}")
addUnitTest(test-qpsolver    "solver" "${solverAvailable}")

## Compare LSSOL and QPOASES.
addUnitTest(test-all-solvers "solver" "${solverAvailable}")

if(WITH_QPOASES)
  addBench(test-humanoid  "mpc-walkgen;timer" "")
  addBench(test-zebulon  "mpc-walkgen;timer" "")
endif()


# a simple execution of the walkgen.
if(WITH_LSSOL AND WITH_QPOASES)
  addBench(bench-solvers-humanoid "mpc-walkgen;timer" "")
  addBench(bench-solvers-zebulon "mpc-walkgen;timer" "")
endif()
