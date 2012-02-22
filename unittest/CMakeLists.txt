INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/include-private)
INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src)

# test the lssol solver
ADD_EXECUTABLE(test-lssol test-lssol.cpp)
PKG_CONFIG_USE_DEPENDENCY(test-lssol lssol)
ADD_TEST(test-lssol test-lssol)

# test the rotation of the cholesky matrix
ADD_EXECUTABLE(test-rotation test-rotation.cpp
	../include-private/mpc-walkgen/tools.h
	../src/walkgen/tools.cpp
	tools-rotation.h
	tools-rotation.cpp
)
ADD_TEST(test-rotation test-rotation)


ADD_EXECUTABLE(test-qpsolver test-qpsolver.cpp
	../src/walkgen/qp-solver.cpp
	../src/walkgen/qp-solvers/lssol-solver.cpp	
	../src/walkgen/qp-matrix.cpp
	../src/walkgen/convex-hull.cpp
	../src/sharedpgtypes.cpp
)	
PKG_CONFIG_USE_DEPENDENCY(test-qpsolver lssol)
ADD_TEST(test-qpsolver test-qpsolver)

# a simple execution of the walkgen. For link reasons,
#  it only works with unix systems.
IF(UNIX AND NOT APPLE)
  ADD_EXECUTABLE(test-walkgen test-walkgen.cpp)	
  ADD_DEPENDENCIES(test-walkgen mpc-walkgen)
  TARGET_LINK_LIBRARIES(test-walkgen mpc-walkgen)
  ADD_TEST(test-walkgen test-walkgen)
ENDIF(UNIX AND NOT APPLE)
