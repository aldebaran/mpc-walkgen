include_directories(BEFORE ${PROJECT_SOURCE_DIR}/src)

qi_create_gtest(test-lip-model
  SRC ./test-lip-model.cpp ../src/model/lip_model.cpp ../src/tools.cpp
  TIMEOUT 1
)

qi_create_gtest(test-base-model
  SRC ./test-base-model.cpp ../src/model/zebulon_base_model.cpp ../src/tools.cpp
  TIMEOUT 1
)

qi_create_gtest(test-base-velocity-tracking-objective
  SRC ./test-base-velocity-tracking-objective
      ../src/model/zebulon_base_model.cpp
      ../src/tools.cpp
      ../src/function/zebulon_base_velocity_tracking_objective.cpp
  TIMEOUT 1
)

qi_create_gtest(test-base-position-tracking-objective
  SRC ./test-base-position-tracking-objective
      ../src/model/zebulon_base_model.cpp
      ../src/tools.cpp
      ../src/function/zebulon_base_position_tracking_objective.cpp
  TIMEOUT 1
)

qi_create_gtest(test-jerk-minimization-objective
  SRC ./test-jerk-minimization-objective
      ../src/model/zebulon_base_model.cpp
      ../src/model/lip_model.cpp
      ../src/tools.cpp
      ../src/function/zebulon_jerk_minimization_objective.cpp
  TIMEOUT 1
)

qi_create_gtest(test-cop-centering-objective
  SRC ./test-cop-centering-objective.cpp
      ../src/model/zebulon_base_model.cpp
      ../src/model/lip_model.cpp
      ../src/tools.cpp
      ../src/function/zebulon_cop_centering_objective.cpp
  TIMEOUT 1
)

qi_create_gtest(test-cop-constraint
  SRC ./test-cop-constraint.cpp
      ../src/model/zebulon_base_model.cpp
      ../src/model/lip_model.cpp
      ../src/tools.cpp
      ../src/function/zebulon_cop_constraint.cpp
  TIMEOUT 1
)

qi_create_gtest(test-com-constraint
  SRC ./test-com-constraint.cpp
      ../src/model/zebulon_base_model.cpp
      ../src/model/lip_model.cpp
      ../src/tools.cpp
      ../src/function/zebulon_com_constraint.cpp
  TIMEOUT 1
)

qi_create_gtest(test-base-motion-constraint
  SRC ./test-base-motion-constraint.cpp
      ../src/model/zebulon_base_model.cpp
      ../src/tools.cpp
      ../src/function/zebulon_base_motion_constraint.cpp
  TIMEOUT 1
)

qi_create_gtest(test-qpoases-solver
  SRC ./test-qpoases-solver.cpp
      ../src/solver/qpoases_solver.cpp
  DEPENDS QPOASESfloat
  TIMEOUT 1
)

qi_create_bin(zebulon-walkgen-bin
  SRC
./zebulon-walkgen-bin.cpp
  DEPENDS mpc-walkgen QPOASESfloat
  NO_INSTALL
)
