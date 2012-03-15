# test the rotation of the cholesky matrix
qi_create_test(test-lssol SRC test-lssol.cpp DEPENDS LSSOL)
target_link_libraries(test-lssol gfortran)

qi_create_test(test-qpoases SRC test-qpoases.cpp DEPENDS qpoases)

qi_create_test(test-rotation
    SRC
  test-rotation.cpp
  ../src/walkgen/tools.cpp
  tools-rotation.cpp
)

if ("ON")

  set_source_files_properties(
    test-qpsolver.cpp
    bench-qpsolver.cpp
    PROPERTIES
    COMPILE_FLAGS -DUSE_QPOASES)
endif ()

qi_create_test(test-qpsolver
    SRC
  test-qpsolver.cpp
  ../src/walkgen/qp-solver.cpp
  ../src/walkgen/qp-solvers/lssol-solver.cpp
  ../src/walkgen/qp-solvers/qpoases-solver.cpp
  ../src/walkgen/qp-matrix.cpp
  ../src/walkgen/convex-hull.cpp
  ../src/sharedpgtypes.cpp
    DEPENDS LSSOL qpoases
)
target_link_libraries(test-qpsolver gfortran)


qi_create_test(bench-qpsolver
    SRC
  bench-qpsolver.cpp
  ../src/walkgen/qp-solver.cpp
  ../src/walkgen/qp-solvers/lssol-solver.cpp
  ../src/walkgen/qp-solvers/qpoases-solver.cpp
  ../src/walkgen/qp-matrix.cpp
  ../src/walkgen/convex-hull.cpp
  ../src/sharedpgtypes.cpp
    DEPENDS mpc-walkgen LSSOL qpoases
)
target_link_libraries(bench-qpsolver gfortran)


qi_create_test(test-all-solvers
    SRC
        test-all-solvers.cpp
        ../src/walkgen/qp-solver.cpp
        ../src/walkgen/qp-solvers/lssol-solver.cpp
        ../src/walkgen/qp-solvers/qpoases-solver.cpp
        ../src/walkgen/qp-matrix.cpp
        ../src/walkgen/convex-hull.cpp
        ../src/sharedpgtypes.cpp
    DEPENDS LSSOL qpoases
)
target_link_libraries(test-all-solvers gfortran)





IF(UNIX AND NOT APPLE)
  qi_create_test(test-walkgen
      SRC
    test-walkgen.cpp
      DEPENDS mpc-walkgen
  )
  target_link_libraries(test-walkgen gfortran)


  qi_create_test(bench-solvers
      SRC
    bench-solvers.cpp
      DEPENDS mpc-walkgen
  )
  target_link_libraries(test-walkgen gfortran)
ENDIF(UNIX AND NOT APPLE)
