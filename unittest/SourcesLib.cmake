INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/include-private)
INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src)

## test the rotation of the cholesky matrix
set(test-rotation_SRC
	test-rotation.cpp
	../include-private/mpc-walkgen/tools.h
	../src/walkgen/tools.cpp
	../src/walkgen/gettimeofday.cpp
	tools-rotation.h
	tools-rotation.cpp
)

## Testing the solvers
# test the lssol solver

set(test-lssol_SRC test-lssol.cpp)

# test the qpoases solver
set(test-qpoases_SRC test-qpoases.cpp)

## Testing all the solvers
set(test-qpsolver_SRC
	test-qpsolver.cpp
	../src/walkgen/qp-solver.cpp
        ../src/walkgen/qp-matrix.cpp
        ../src/walkgen/convex-hull.cpp
        ../src/sharedpgtypes.cpp
)

#
set(test-all-solvers_SRC
        test-all-solvers.cpp
        ../src/walkgen/qp-solver.cpp
        ../src/walkgen/qp-matrix.cpp
        ../src/walkgen/convex-hull.cpp
        ../src/sharedpgtypes.cpp
)

#
set(bench-qpsolver_SRC
  bench-qpsolver.cpp
  ../src/walkgen/qp-solver.cpp
  ../src/walkgen/qp-matrix.cpp
  ../src/walkgen/convex-hull.cpp
  ../src/walkgen/mpc-debug.cpp
  ../src/walkgen/gettimeofday.cpp
  ../src/sharedpgtypes.cpp
)

if(LSSOL_FOUND)
  list(APPEND bench-qpsolver_SRC ../src/walkgen/qp-solvers/lssol-solver.cpp)
  list(APPEND test-all-solvers_SRC ../src/walkgen/qp-solvers/lssol-solver.cpp)
  list(APPEND test-qpsolver_SRC ../src/walkgen/qp-solvers/lssol-solver.cpp)
endif(LSSOL_FOUND)

if(QPOASES_FOUND)
  list(APPEND bench-qpsolver_SRC ../src/walkgen/qp-solvers/qpoases-solver.cpp)
  list(APPEND test-all-solvers_SRC ../src/walkgen/qp-solvers/qpoases-solver.cpp)
  list(APPEND test-qpsolver_SRC ../src/walkgen/qp-solvers/qpoases-solver.cpp)
endif(QPOASES_FOUND)

# tests of the walkgen
set(test-walkgen_SRC  test-walkgen.cpp)
set(bench-solvers_SRC  bench-solvers.cpp)
