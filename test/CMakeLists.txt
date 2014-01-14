include_directories(BEFORE ${PROJECT_SOURCE_DIR}/src)

qi_create_gtest(test-lip-model
  SRC ./test-lip-model.cpp
../src/model/lip_model.cpp
../src/tools.cpp
../src/type.cpp
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-base-model
  SRC ./test-zebulon-base-model.cpp
../src/model/zebulon_base_model.cpp
../src/tools.cpp
../src/type.cpp
  TIMEOUT 1
)

qi_create_gtest(test-humanoid-foot-model
  SRC ./test-humanoid-foot-model.cpp
../src/model/humanoid_foot_model.cpp
../src/tools.cpp
../src/type.cpp
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-base-velocity-tracking-objective
  SRC ./test-zebulon-base-velocity-tracking-objective
      ../src/model/zebulon_base_model.cpp
      ../src/tools.cpp
      ../src/type.cpp
      ../src/function/zebulon_base_velocity_tracking_objective.cpp
  TIMEOUT 1
)

qi_create_gtest(test-humanoid-lip-com-velocity-tracking-objective
  SRC ./test-humanoid-lip-com-velocity-tracking-objective
      ../src/humanoid_feet_supervisor.cpp
      ../src/model/humanoid_foot_model.cpp
      ../src/model/lip_model.cpp
      ../src/tools.cpp
      ../src/type.cpp
      ../src/function/humanoid_lip_com_velocity_tracking_objective.cpp
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-base-position-tracking-objective
  SRC ./test-zebulon-base-position-tracking-objective
      ../src/model/zebulon_base_model.cpp
      ../src/tools.cpp
      ../src/type.cpp
      ../src/function/zebulon_base_position_tracking_objective.cpp
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-jerk-minimization-objective
  SRC ./test-zebulon-jerk-minimization-objective
      ../src/model/zebulon_base_model.cpp
      ../src/model/lip_model.cpp
      ../src/tools.cpp
      ../src/type.cpp
      ../src/function/zebulon_jerk_minimization_objective.cpp
  TIMEOUT 1
)

qi_create_gtest(test-humanoid-lip-com-jerk-minimization-objective
  SRC ./test-humanoid-lip-com-jerk-minimization-objective
      ../src/humanoid_feet_supervisor.cpp
      ../src/model/humanoid_foot_model.cpp
      ../src/model/lip_model.cpp
      ../src/tools.cpp
      ../src/type.cpp
      ../src/function/humanoid_lip_com_jerk_minimization_objective.cpp
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-cop-centering-objective
  SRC ./test-zebulon-cop-centering-objective.cpp
      ../src/model/zebulon_base_model.cpp
      ../src/model/lip_model.cpp
      ../src/tools.cpp
      ../src/type.cpp
      ../src/function/zebulon_cop_centering_objective.cpp
  TIMEOUT 1
)

qi_create_gtest(test-humanoid-cop-centering-objective
  SRC ./test-humanoid-cop-centering-objective.cpp
      ../src/humanoid_feet_supervisor.cpp
      ../src/model/humanoid_foot_model.cpp
      ../src/model/lip_model.cpp
      ../src/tools.cpp
      ../src/type.cpp
      ../src/function/humanoid_cop_centering_objective.cpp
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-cop-constraint
  SRC ./test-zebulon-cop-constraint.cpp
      ../src/model/zebulon_base_model.cpp
      ../src/model/lip_model.cpp
      ../src/tools.cpp
      ../src/type.cpp
      ../src/function/zebulon_cop_constraint.cpp
  TIMEOUT 1
)

qi_create_gtest(test-humanoid-cop-constraint
  SRC ./test-humanoid-cop-constraint.cpp
      ../src/humanoid_feet_supervisor.cpp
      ../src/model/humanoid_foot_model.cpp
      ../src/model/lip_model.cpp
      ../src/tools.cpp
      ../src/type.cpp
      ../src/function/humanoid_cop_constraint.cpp
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-com-constraint
  SRC ./test-zebulon-com-constraint.cpp
      ../src/model/zebulon_base_model.cpp
      ../src/model/lip_model.cpp
      ../src/tools.cpp
      ../src/type.cpp
      ../src/function/zebulon_com_constraint.cpp
  TIMEOUT 1
)

qi_create_gtest(test-humanoid-foot-constraint
  SRC ./test-humanoid-foot-constraint.cpp
      ../src/humanoid_feet_supervisor.cpp
      ../src/model/humanoid_foot_model.cpp
      ../src/model/lip_model.cpp
      ../src/tools.cpp
      ../src/type.cpp
      ../src/function/humanoid_foot_constraint.cpp
  TIMEOUT 1
)

qi_create_gtest(test-zebulon-base-motion-constraint
  SRC ./test-zebulon-base-motion-constraint.cpp
      ../src/model/zebulon_base_model.cpp
      ../src/tools.cpp
      ../src/type.cpp
      ../src/function/zebulon_base_motion_constraint.cpp
  TIMEOUT 1
)

qi_create_gtest(test-qpoases-solver
  SRC ./test-qpoases-solver.cpp
      ../src/qpsolverfactory.h
      ../src/qpsolverfactory.cpp
      ../src/type.h
  DEPENDS mpc-walkgen
          qpOASESfloat
  TIMEOUT 1
)

qi_create_bin(zebulon-walkgen-bin
  SRC
./zebulon-walkgen-bin.cpp
  DEPENDS mpc-walkgen QPOASESfloat
  NO_INSTALL
)

qi_create_gtest(test-convex-polygon-function
  SRC ./test-convex-polygon-function.cpp
../src/type.cpp
  TIMEOUT 1
)

qi_create_gtest(test-interpolation-function
  SRC ./test-interpolation-function.cpp
../src/type.cpp
../src/tools.cpp
  TIMEOUT 1
)
