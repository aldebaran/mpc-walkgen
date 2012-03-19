# Load the file listing the sources required for mpc-walkgen
INCLUDE(SourcesLib.cmake)

INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/include-private)
INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src)

# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

ADD_LIBRARY(mpc-walkgen SHARED ${mpc-walkgen_SRC})
SET_TARGET_PROPERTIES(mpc-walkgen PROPERTIES SOVERSION ${PROJECT_VERSION})
INSTALL(TARGETS mpc-walkgen DESTINATION lib)

if(LSSOL_FOUND)
 PKG_CONFIG_USE_DEPENDENCY(mpc-walkgen lssol)
endif(LSSOL_FOUND)

if(QPOASES_FOUND)
 PKG_CONFIG_USE_DEPENDENCY(mpc-walkgen qpoases)
endif(QPOASES_FOUND)
