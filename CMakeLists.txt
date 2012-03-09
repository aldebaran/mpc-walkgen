project(mpc-walkgen)
cmake_minimum_required(VERSION 2.8)
find_package(qibuild)

#SET(CXX_DISABLE_WERROR True)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules")
INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/include-private)
INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src)

SET(SOURCES
 src/sharedpgtypes.cpp 
 
 src/walkgen/convex-hull.cpp 
 src/walkgen/orientations-preview.cpp 

 
 src/walkgen/qp-solver.cpp
 src/walkgen/qp-solvers/lssol-solver.cpp
 src/walkgen/qp-solvers/qpoases-solver.cpp
 
 src/walkgen/state-solver.cpp
 src/walkgen/state-solvers/fsm-solver.cpp 
 
 src/walkgen/rigid-body-system.cpp
 src/walkgen/rigid-body.cpp
 src/walkgen/rigid-bodies/com-body.cpp
 src/walkgen/rigid-bodies/foot-body.cpp
 
 src/walkgen/qp-matrix.cpp
 src/walkgen/qp-generator.cpp
 src/walkgen/qp-preview.cpp
 
 src/walkgen/interpolation.cpp
 src/walkgen/tools.cpp
 src/walkgen/types.cpp
 src/walkgen/walkgen.cpp
 
 src/walkgen/mpc-debug.cpp
 )

qi_create_lib(mpc-walkgen SRC "${SOURCES}")
qi_use_lib(mpc-walkgen lssol)
target_link_libraries(mpc-walkgen gfortran)
qi_use_lib(mpc-walkgen qpoases)
qi_use_lib(mpc-walkgen boost)
qi_use_lib(mpc-walkgen eigen3)
qi_stage_lib(mpc-walkgen INCLUDE_DIRS include/mpc-walkgen)
#SET_TARGET_PROPERTIES(mpc-walkgen PROPERTIES SOVERSION ${PROJECT_VERSION})

# Define subdirectories to explore for cmake
ADD_SUBDIRECTORY(unittest)

# Add a simple test:
enable_testing()

