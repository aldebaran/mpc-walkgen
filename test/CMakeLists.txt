# common stuff
qi_create_gtest(test-qpoases-solver
  SRC ./test-qpoases-solver.cpp
  DEPENDS mpc-walkgen
          qpOASESfloat
          boost
  TIMEOUT 1
)

qi_create_gtest(test-convex-polygon-function
  SRC ./test-convex-polygon-function.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-interpolation-function
  SRC ./test-interpolation-function.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-lip-model
  SRC ./test-lip-model.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

# zebulon stuff
qi_create_gtest(test-zebulon-base-model
  SRC ./test-zebulon-base-model.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-base-velocity-tracking-objective
  SRC ./test-zebulon-base-velocity-tracking-objective.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-base-position-tracking-objective
  SRC ./test-zebulon-base-position-tracking-objective.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-jerk-minimization-objective
  SRC ./test-zebulon-jerk-minimization-objective.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-cop-centering-objective
  SRC ./test-zebulon-cop-centering-objective.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-cop-constraint
  SRC ./test-zebulon-cop-constraint.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-com-constraint
  SRC ./test-zebulon-com-constraint.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-base-motion-constraint
  SRC ./test-zebulon-base-motion-constraint.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-tilt-motion-constraint
  SRC ./test-zebulon-tilt-motion-constraint.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_bin(zebulon-walkgen-bin
  SRC ./zebulon-walkgen-bin.cpp
  DEPENDS mpc-walkgen
  NO_INSTALL
)

qi_create_bin(trajectory-walkgen-bin
  SRC ./trajectory-walkgen-bin.cpp
  DEPENDS mpc-walkgen
  NO_INSTALL
)

qi_create_bin(humanoid-walkgen-bin
  SRC ./humanoid-walkgen-bin.cpp
  DEPENDS mpc-walkgen
  NO_INSTALL
)
# humanoid stuff
# mostly smoke tests that only help ensure the templates keep building
qi_create_gtest(test-humanoid-foot-model
  SRC ./test-humanoid-foot-model.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-humanoid-feet-supervisor
  SRC ./test-humanoid-feet-supervisor.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-humanoid-lip-com-velocity-tracking-objective
  SRC ./test-humanoid-lip-com-velocity-tracking-objective.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-humanoid-lip-com-jerk-minimization-objective
  SRC ./test-humanoid-lip-com-jerk-minimization-objective.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-humanoid-cop-centering-objective
  SRC ./test-humanoid-cop-centering-objective.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-humanoid-foot-constraint
  SRC ./test-humanoid-foot-constraint.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)

qi_create_gtest(test-humanoid-cop-constraint
  SRC ./test-humanoid-cop-constraint.cpp
  DEPENDS mpc-walkgen
  TIMEOUT 1
)
