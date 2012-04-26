INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/include-private)
INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src)

## test the rotation of the cholesky matrix
set(test-rotation_SRC
	test-rotation.cpp
	../src/walkgen/tools.h
	../src/walkgen/tools.cpp
	tools-rotation.h
	tools-rotation.cpp
)

## Testing the solvers
# test the lssol solver

set(test-lssol_SRC test-lssol.cpp)

# test the qpoases solver
set(test-qpoases_SRC test-qpoases.cpp)


## Static libraries
# for the solver
set(solver_SRC
  ../src/common/qp-solver.cpp
  ../src/common/qp-matrix.cpp
  ../src/common/qp-vector.cpp
  ../src/walkgen/convex-hull.cpp
  ../src/sharedpgtypes.cpp
  ../src/sharedpgtypes-humanoid.cpp
  ../src/walkgen-abstract.cpp
  ../src/walkgen-abstract-humanoid.cpp
)

## Testing all the solvers
set(test-qpsolver_SRC 
	test-qpsolver.cpp
	../src/common/qp-solver.cpp
	../src/common/qp-matrix.cpp
 	../src/walkgen/convex-hull.cpp
 	../src/sharedpgtypes.cpp
        ../src/sharedpgtypes-humanoid.cpp
        ../src/walkgen-abstract.cpp
        ../src/walkgen-abstract-humanoid.cpp
)

# 
set(test-all-solvers_SRC 
        test-all-solvers.cpp
        ../src/common/qp-solver.cpp
        ../src/common/qp-solvers/lssol-solver.cpp
        ../src/common/qp-solvers/qpoases-solver.cpp
        ../src/common/qp-matrix.cpp
        ../src/walkgen/convex-hull.cpp
        ../src/sharedpgtypes.cpp
        ../src/sharedpgtypes-humanoid.cpp
        ../src/walkgen-abstract.cpp
        ../src/walkgen-abstract-humanoid.cpp
)

# 
set(bench-qpsolver_SRC 
  bench-qpsolver.cpp
  ../src/common/qp-solver.cpp
  ../src/common/qp-matrix.cpp
  ../src/walkgen/convex-hull.cpp
  ../src/sharedpgtypes.cpp
  ../src/sharedpgtypes-humanoid.cpp
  ../src/walkgen-abstract.cpp
  ../src/walkgen-abstract-humanoid.cpp

)
if(LSSOL_FOUND)
  list(APPEND solver_SRC ../src/common/qp-solvers/lssol-solver.cpp)
endif(LSSOL_FOUND)

if(QPOASES_FOUND)
  list(APPEND solver_SRC ../src/common/qp-solvers/qpoases-solver.cpp)
endif(QPOASES_FOUND)

# for the timer
set(timer_SRC
  ../src/walkgen/mpc-debug.cpp
  ../src/walkgen/gettimeofday.cpp
)

## Testing all the solvers
set(test-qpsolver_SRC test-qpsolver.cpp)

#
set(test-all-solvers_SRC test-all-solvers.cpp)

#
set(bench-qpsolver_SRC bench-qpsolver.cpp)

# tests of the walkgen
set(test-walkgen_SRC  test-walkgen.cpp ../src/walkgen/mpc-debug.cpp)
set(bench-solvers_SRC  bench-solvers.cpp)
