INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src)

## test the rotation of the cholesky matrix
set(test-rotation_SRC
	test-rotation.cpp
	../src/common/tools.h
	../src/common/tools.cpp
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
  ../src/humanoid/types.cpp
  ../src/humanoid/sharedpgtypes.cpp
  ../src/humanoid/walkgen-abstract.cpp
)

if(WITH_LSSOL)
  list(APPEND solver_SRC ../src/common/qp-solvers/lssol-solver.cpp)
endif()

if(WITH_QPOASES)
  list(APPEND solver_SRC ../src/common/qp-solvers/qpoases-solver.cpp)
endif()

## Testing all the solvers
set(test-qpsolver_SRC 
  test-qpsolver.cpp
)

# 
set(test-all-solvers_SRC 
  test-all-solvers.cpp
)

# 
set(bench-qpsolver_SRC 
  bench-qpsolver.cpp
)

# for the timer
set(timer_SRC
  ../src/common/mpc-debug.cpp
  ../src/common/gettimeofday.cpp
)

# tests of the walkgen
set(test-humanoid_SRC  test-humanoid.cpp ../src/common/mpc-debug.cpp)
set(test-zebulon_SRC  test-zebulon.cpp ../src/common/mpc-debug.cpp)

set(bench-solvers-humanoid_SRC  bench-solvers-humanoid.cpp ../src/common/mpc-debug.cpp)
set(bench-solvers-zebulon_SRC  bench-solvers-zebulon.cpp ../src/common/mpc-debug.cpp)
