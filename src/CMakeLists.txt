# Copyright 2010, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of jrl-walkgen.
# jrl-walkgen is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# jrl-walkgen is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# jrl-walkgen. If not, see <http://www.gnu.org/licenses/>.

INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/include-private)
INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src)

# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# add flag to compile qld.cc
IF(WIN32)
ADD_DEFINITIONS("/D __STDC__")
ENDIF(WIN32)

SET(SOURCES
 sharedpgtypes.cpp 
 
 walkgen/orientations-preview.cpp 

 
 walkgen/qp-solver.cpp
 walkgen/qp-solvers/lssol-solver.cpp
 
 walkgen/state-solver.cpp
 walkgen/state-solvers/fsm-solver.cpp 
 
 walkgen/rigid-body-system.cpp
 walkgen/rigid-body.cpp
 walkgen/rigid-bodies/com-body.cpp
 walkgen/rigid-bodies/foot-body.cpp
 
 walkgen/qp-matrix.cpp
 walkgen/qp-generator.cpp
 walkgen/qp-preview.cpp
 
 walkgen/interpolation.cpp
 walkgen/tools.cpp
 walkgen/types.cpp
 walkgen/walkgen.cpp
 
 walkgen/mpc-debug.cpp
 )

ADD_LIBRARY(mpc-walkgen SHARED ${SOURCES})
SET_TARGET_PROPERTIES(mpc-walkgen PROPERTIES SOVERSION ${PROJECT_VERSION})
INSTALL(TARGETS mpc-walkgen DESTINATION lib)

PKG_CONFIG_USE_DEPENDENCY(mpc-walkgen lssol)
