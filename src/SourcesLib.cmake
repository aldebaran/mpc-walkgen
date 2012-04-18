SET(mpc-walkgen_SRC

 sharedpgtypes.cpp 
 walkgen-abstract.cpp
 walkgen-abstract-humanoid.cpp
 
 walkgen/convex-hull.cpp 
 walkgen/orientations-preview.cpp 


 walkgen/qp-solver.cpp

 walkgen/state-solver.cpp
 walkgen/state-solvers/fsm-solver.cpp

 walkgen/rigid-body-system.cpp
 walkgen/rigid-body.cpp
 walkgen/rigid-bodies/com-body.cpp
 walkgen/rigid-bodies/foot-body.cpp

 walkgen/qp-matrix.cpp
 walkgen/qp-vector.cpp
 walkgen/qp-generator.cpp
 walkgen/qp-preview.cpp

 walkgen/interpolation.cpp
 walkgen/tools.cpp
 walkgen/types.cpp

 walkgen/walkgen-humanoid.cpp

 walkgen/gettimeofday.cpp

 walkgen/mpc-debug.cpp
 walkgen/perturbation.cpp
)

if(LSSOL_FOUND)
  list(APPEND mpc-walkgen_SRC
    walkgen/qp-solvers/lssol-solver.cpp
  )
endif(LSSOL_FOUND)

if(QPOASES_FOUND)
  list(APPEND mpc-walkgen_SRC
    walkgen/qp-solvers/qpoases-solver.cpp
  )
endif(QPOASES_FOUND)

