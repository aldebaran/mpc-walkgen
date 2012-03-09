# test the rotation of the cholesky matrix
qi_create_test(test-lssol SRC test-lssol.cpp DEPENDS LSSOL)
target_link_libraries(test-lssol gfortran)

qi_create_test(test-rotation 
    SRC 
	test-rotation.cpp 
	../src/walkgen/tools.cpp 
	tools-rotation.cpp
)


qi_create_test(test-qpsolver 
    SRC 
	test-qpsolver.cpp 
	../src/walkgen/qp-solver.cpp
	../src/walkgen/qp-solvers/lssol-solver.cpp	
	../src/walkgen/qp-matrix.cpp
	../src/walkgen/convex-hull.cpp
	../src/sharedpgtypes.cpp
    DEPENDS LSSOL
)
target_link_libraries(test-qpsolver gfortran)



IF(UNIX AND NOT APPLE)
	qi_create_test(test-walkgen 
	    SRC 
		test-walkgen.cpp 
	    DEPENDS mpc-walkgen
	)
	target_link_libraries(test-walkgen gfortran)
ENDIF(UNIX AND NOT APPLE)
