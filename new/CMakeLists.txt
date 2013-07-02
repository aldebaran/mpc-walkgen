SET(mpc-walkgen-new_SRC
src/model/zebulon_base_model.h
src/model/zebulon_base_model.cpp
src/model/lip_model.h
src/model/lip_model.cpp
src/function/zebulon_base_velocity_tracking_objective.h
src/function/zebulon_base_velocity_tracking_objective.cpp
src/function/zebulon_base_position_tracking_objective.h
src/function/zebulon_base_position_tracking_objective.cpp
src/function/zebulon_jerk_minimization_objective.h
src/function/zebulon_jerk_minimization_objective.cpp
src/function/zebulon_cop_centering_objective.h
src/function/zebulon_cop_centering_objective.cpp
src/function/zebulon_cop_constraint.h
src/function/zebulon_cop_constraint.cpp
src/function/zebulon_com_constraint.h
src/function/zebulon_com_constraint.cpp
src/function/zebulon_base_motion_constraint.h
src/function/zebulon_base_motion_constraint.cpp
src/solver/qpoases_solver.h
src/solver/qpoases_solver.cpp
src/type.h
src/tools.h
src/tools.cpp
src/zebulon_walkgen.h
src/zebulon_walkgen.cpp
)

SET(mpc-walkgen-new_PUBLIC_HEADERS
mpc-walkgen/zebulon_walkgen_abstract.h
mpc-walkgen/zebulon_walkgen_abstract.cpp
mpc-walkgen/shared_type.h
mpc-walkgen/api.h
)

qi_sanitize_compile_flags(HIDDEN_SYMBOLS)

qi_create_lib(mpc-walkgen-new SHARED ${mpc-walkgen-new_SRC} ${mpc-walkgen-new_PUBLIC_HEADERS})
qi_use_lib(mpc-walkgen-new eigen3 QI)
qi_use_lib(mpc-walkgen-new QPOASESfloat)
qi_stage_lib(mpc-walkgen-new)

qi_install_header(${mpc-walkgen-new_PUBLIC_HEADERS} SUBFOLDER mpc-walkgen)

add_subdirectory(test)


