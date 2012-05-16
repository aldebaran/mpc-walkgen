# Load the file listing the sources required for each unit test.
include(SourcesLib.cmake)

INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src)

# define a macro wrapper to add an unit test.
#  name is the name of the unit test.
#  internal_dep indicates if the internal dep (aka does that depends on mpc-walkgen?)
#  external_dep lists the external dependencies (such as the solver)
macro(addUnitTest name internal_dep external_dep)
  ADD_EXECUTABLE(${name}  ${${name}_SRC})
  ADD_TEST(${name} ${name})

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
  addUnitTest(test-humanoid  "mpc-walkgen;timer" "")
  addUnitTest(test-zebulon  "mpc-walkgen;timer" "")
endif()


# a simple execution of the walkgen.
if(WITH_LSSOL AND WITH_QPOASES)
  addUnitTest(bench-solvers-humanoid "mpc-walkgen;timer" "")
  addUnitTest(bench-solvers-zebulon "mpc-walkgen;timer" "")
endif()
