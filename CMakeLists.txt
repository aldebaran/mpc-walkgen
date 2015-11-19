# Copyright 2012, Olivier Stasse, JRL, CNRS/AIST
# Requires at least CMake 2.6 to configure the package.
cmake_minimum_required(VERSION 2.8)
project(mpc-walkgen)
enable_testing()
find_package(qibuild)

qi_sanitize_compile_flags(HIDDEN_SYMBOLS)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_INITIALIZE_MATRICES_BY_NAN=1")
endif()

add_subdirectory(qpsolver)

SET(mpc-walkgen_common_PUBLIC_HEADERS
mpc-walkgen/api.h
mpc-walkgen/constant.h
mpc-walkgen/convexpolygon.h
mpc-walkgen/interpolator.h
mpc-walkgen/lineardynamic.h
mpc-walkgen/model/lip_model.h
mpc-walkgen/qpsolverfactory.h
mpc-walkgen/tools.h
mpc-walkgen/type.h
)
SET(mpc-walkgen_humanoid_PUBLIC_HEADERS
mpc-walkgen/function/humanoid_cop_centering_objective.h
mpc-walkgen/function/humanoid_cop_constraint.h
mpc-walkgen/function/humanoid_foot_constraint.h
mpc-walkgen/function/humanoid_lip_com_jerk_minimization_objective.h
mpc-walkgen/function/humanoid_lip_com_velocity_tracking_objective.h
mpc-walkgen/humanoid_feet_supervisor.h
mpc-walkgen/humanoid_walkgen.h
mpc-walkgen/humanoid_walkgen_type.h
mpc-walkgen/model/humanoid_foot_model.h
)
SET(mpc-walkgen_zebulon_PUBLIC_HEADERS
mpc-walkgen/function/zebulon_base_velocity_tracking_objective.h
mpc-walkgen/function/zebulon_base_position_tracking_objective.h
mpc-walkgen/function/zebulon_jerk_minimization_objective.h
mpc-walkgen/function/zebulon_tilt_minimization_objective.h
mpc-walkgen/function/zebulon_tilt_velocity_minimization_objective.h
mpc-walkgen/function/zebulon_cop_centering_objective.h
mpc-walkgen/function/zebulon_com_centering_objective.h
mpc-walkgen/function/zebulon_cop_constraint.h
mpc-walkgen/function/zebulon_com_constraint.h
mpc-walkgen/function/zebulon_base_motion_constraint.h
mpc-walkgen/function/zebulon_tilt_motion_constraint.h
mpc-walkgen/model/zebulon_base_model.h
mpc-walkgen/zebulon_walkgen.h
mpc-walkgen/zebulon_walkgen_type.h
)
SET(mpc-walkgen_trajectory_PUBLIC_HEADERS
mpc-walkgen/function/trajectory_motion_constraint.h
mpc-walkgen/function/trajectory_velocity_tracking_objective.h
mpc-walkgen/function/trajectory_position_tracking_objective.h
mpc-walkgen/function/trajectory_jerk_minimization_objective.h
mpc-walkgen/model/no_dynamic_model.h
mpc-walkgen/trajectory_walkgen.h
mpc-walkgen/trajectory_walkgen_type.h
)
SET(mpc-walkgen_PUBLIC_HEADERS
${mpc-walkgen_common_PUBLIC_HEADERS}
${mpc-walkgen_humanoid_PUBLIC_HEADERS}
${mpc-walkgen_zebulon_PUBLIC_HEADERS}
${mpc-walkgen_trajectory_PUBLIC_HEADERS})


SET(mpc-walkgen_SRC
src/convexpolygon.cpp
src/interpolator.cpp
src/lineardynamic.cpp
src/macro.h
src/model/lip_model.cpp
src/qpsolverfactory.cpp
src/tools.cpp

src/function/humanoid_lip_com_velocity_tracking_objective.cpp
src/function/humanoid_lip_com_jerk_minimization_objective.cpp
src/function/humanoid_cop_centering_objective.cpp
src/function/humanoid_cop_constraint.cpp
src/function/humanoid_foot_constraint.cpp
src/humanoid_walkgen.cpp
src/humanoid_feet_supervisor.cpp
src/model/humanoid_foot_model.cpp

src/function/zebulon_base_motion_constraint.cpp
src/function/zebulon_base_position_tracking_objective.cpp
src/function/zebulon_base_velocity_tracking_objective.cpp
src/function/zebulon_com_centering_objective.cpp
src/function/zebulon_com_constraint.cpp
src/function/zebulon_cop_centering_objective.cpp
src/function/zebulon_cop_constraint.cpp
src/function/zebulon_jerk_minimization_objective.cpp
src/function/zebulon_tilt_minimization_objective.cpp
src/function/zebulon_tilt_motion_constraint.cpp
src/function/zebulon_tilt_velocity_minimization_objective.cpp
src/model/zebulon_base_model.cpp
src/zebulon_walkgen.cpp

src/function/trajectory_position_tracking_objective.cpp
src/function/trajectory_velocity_tracking_objective.cpp
src/function/trajectory_motion_constraint.cpp
src/function/trajectory_jerk_minimization_objective.cpp
src/model/no_dynamic_model.cpp
src/trajectory_walkgen.cpp
)

qi_create_lib(mpc-walkgen SHARED
              ${mpc-walkgen_PUBLIC_HEADERS}
              ${mpc-walkgen_SRC})
qi_use_lib(mpc-walkgen
           eigen3 QI boost
           mpc-walkgen_qpsolver
           mpc-walkgen_qpsolver_qpoases_double
           mpc-walkgen_qpsolver_qpoases_float)
qi_stage_lib(mpc-walkgen)

qi_install_header(${mpc-walkgen_PUBLIC_HEADERS} KEEP_RELATIVE_PATHS)

add_subdirectory(test)
