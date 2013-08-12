# Copyright 2012, Olivier Stasse, JRL, CNRS/AIST
# Requires at least CMake 2.6 to configure the package.
cmake_minimum_required(VERSION 2.8)
project(mpc-walkgen)
enable_testing()
find_package(qibuild)

qi_add_optional_package(QPOASES)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

SET(mpc-walkgen_SRC
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
src/function/zebulon_tilt_minimization_objective.h
src/function/zebulon_tilt_minimization_objective.cpp
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
src/zebulon_walkgen_abstract.cpp
src/shared_type.cpp
)

SET(mpc-walkgen_PUBLIC_HEADERS
mpc-walkgen/zebulon_walkgen_abstract.h
mpc-walkgen/shared_type.h
mpc-walkgen/api.h
)

qi_sanitize_compile_flags(HIDDEN_SYMBOLS)

qi_create_lib(mpc-walkgen SHARED ${mpc-walkgen_SRC} ${mpc-walkgen_PUBLIC_HEADERS})
qi_use_lib(mpc-walkgen eigen3 QI)
qi_use_lib(mpc-walkgen QPOASESfloat)
qi_stage_lib(mpc-walkgen)

qi_install_header(${mpc-walkgen_PUBLIC_HEADERS} SUBFOLDER mpc-walkgen)

add_subdirectory(test)


