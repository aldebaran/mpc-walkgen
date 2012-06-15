include_directories(BEFORE ${PROJECT_SOURCE_DIR}/src)

# Add static libraries to gather some code used in the unit tests
# and avoid compiling the same files over and over

# the qpsolverhumanoid library holds the QP-solving subset of mpc-walkgen,
# with the types defined for the "humanoid" problem.
set(qpsolverhumanoid_SRC
  ../src/common/qp-solver.cpp
  ../src/common/qp-matrix.cpp
  ../src/common/qp-vector.cpp
  ../src/humanoid/types.cpp
  ../src/humanoid/sharedpgtypes.cpp
  ../src/humanoid/walkgen-abstract.cpp)
if(WITH_LSSOL)
  list(APPEND qpsolverhumanoid_SRC ../src/common/qp-solvers/lssol-solver.cpp)
endif()
if(WITH_QPOASES)
  list(APPEND qpsolverhumanoid_SRC ../src/common/qp-solvers/qpoases-solver.cpp)
endif()
add_library(qpsolverhumanoid STATIC ${qpsolverhumanoid_SRC})

# the timer library holds some timing and benchmark tools.
add_library(timer STATIC
  ../src/common/mpc-debug.cpp
  ../src/common/gettimeofday.cpp)


if(WITH_LSSOL)
  list(APPEND solverAvailable "LSSOL")
endif()

if(WITH_QPOASES)
  list(APPEND solverAvailable "QPOASES")
endif()

############
#Benchs
############
# Testing all the available qp solvers, using the QPSolver API
create_test(bench-qpsolver
  SRC bench/bench-qpsolver.cpp
  INTDEPS qpsolverhumanoid timer
  EXTDEPS ${solverAvailable})

# a simple execution of the walkgen.
if(WITH_LSSOL AND WITH_QPOASES)
  create_test(bench-solvers-humanoid
    SRC bench/bench-solvers-humanoid.cpp ../src/common/mpc-debug.cpp
    INTDEPS mpc-walkgen timer)
  create_test(bench-solvers-zebulon
    SRC bench/bench-solvers-zebulon.cpp ../src/common/mpc-debug.cpp
    INTDEPS mpc-walkgen timer)
endif()



############
#Units tests
############
if(WITH_LSSOL)
  # test the lssol solver, using its native API
  create_test(test-lssol
    SRC unit-test/test-lssol.cpp
    EXTDEPS lssol)
endif()

if(WITH_QPOASES)
  # test the qpOASES solver, using its native API
  create_test(test-qpoases
    SRC unit-test/test-qpoases.cpp
    EXTDEPS QPOASES)
endif()

create_test(test-qpsolver
  SRC unit-test/test-qpsolver.cpp
  INTDEPS qpsolverhumanoid
  EXTDEPS ${solverAvailable})

# Compare LSSOL and qpOASES.
create_test(test-all-solvers
  SRC unit-test/test-all-solvers.cpp
  INTDEPS qpsolverhumanoid
  EXTDEPS ${solverAvailable})

############
#API tests
############





# the "with-solver" subfolder is included once per supported solver. The tests
# defined within it should use the "CURRENT_QPSOLVERTYPE" macro to use the
# correpsonding solver.
# For each inclusion of the folder, a different binary folder is used. It acts
# as a namespace for CMake. This makes it possible to use different values
# for "CURRENT_QPSOLVERTYPE" when calling add_definitions(), without conflicts.
if(WITH_LSSOL)
  set(CURRENT_QPSOLVER "LSSOL")
  add_subdirectory(use-solver use-lssol)
endif()
if(WITH_QPOASES)
   set(CURRENT_QPSOLVER "QPOASES")
  add_subdirectory(use-solver use-qpoases)
endif()
