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
if(LSSOL_FOUND)
  list(APPEND solverAvailable "lssol")
  addUnitTest(test-lssol "" "lssol")
endif(LSSOL_FOUND)


# test the qpoases solver
if(QPOASES_FOUND)
  list(APPEND solverAvailable "QPOASES")
  addUnitTest(test-qpoases "" "QPOASES")
endif(QPOASES_FOUND)


## Testing all the solvers
addUnitTest(bench-qpsolver   "solver;timer" "${solverAvailable}")
addUnitTest(test-qpsolver    "solver" "${solverAvailable}")

## Compare LSSOL and QPOASES.
if(LSSOL_FOUND AND QPOASES_FOUND)
  addUnitTest(test-all-solvers "solver" "${solverAvailable}")
endif(LSSOL_FOUND AND QPOASES_FOUND)

addUnitTest(test-walkgen  "mpc-walkgen" "")

# a simple execution of the walkgen.
if(LSSOL_FOUND AND QPOASES_FOUND)
  addUnitTest(bench-solvers "mpc-walkgen;timer" "")
endif(LSSOL_FOUND AND QPOASES_FOUND)
