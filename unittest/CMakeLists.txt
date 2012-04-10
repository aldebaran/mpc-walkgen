# Load the file listing the sources required for each unit test.
include(SourcesLib.cmake)

if(use_qibuild)

  # define a macro wrapper to add an unit test.
  #  name is the name of the unit test.
  #  internal_dep indicates if the internal dep (aka does that depends on mpc-walkgen?)
  #  external_dep lists the external dependencies (such as the solver)
  #  use_gfortran indicates if it is required to link with gfortran
  macro(addUnitTest name internal_dep external_dep use_gfortran)
    set(dependencies "")

    # list the dependencies - and check if LSSOL is used.
    foreach(dep ${internal_dep} ${external_dep})
      set(dependencies ${dependencies} ${dep})
    endforeach(dep)
    # create the unit test.
    if("${dependencies}" STREQUAL "")
      qi_create_test(${name}  SRC  ${${name}_SRC})
    else()
      qi_create_test(${name}  SRC  ${${name}_SRC}
         DEPENDS ${dependencies})
    endif()

    if(${use_gfortran})
      target_link_libraries(${name} gfortran)
    endif(${use_gfortran})
  endmacro(addUnitTest)


else()

  INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/include-private)
  INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src)

  # define a macro wrapper to add an unit test.
  #  name is the name of the unit test.
  #  internal_dep indicates if the internal dep (aka does that depends on mpc-walkgen?)
  #  external_dep lists the external dependencies (such as the solver)
  #  use_gfortran is not used
  macro(addUnitTest name internal_dep external_dep use_gfortran)
    ADD_EXECUTABLE(${name}  ${${name}_SRC})
    ADD_TEST(${name} ${name})

    foreach(dep ${internal_dep})
      ADD_DEPENDENCIES(${name} ${dep})
      TARGET_LINK_LIBRARIES(${name} ${dep})
    endforeach(dep)

    foreach(dep ${external_dep})
      PKG_CONFIG_USE_DEPENDENCY(${name} ${dep})
    endforeach(dep)
  endmacro(addUnitTest)

endif(use_qibuild)

#### GENERIC PART.
# All the unit tests are created here.

## test the rotation of the cholesky matrix
addUnitTest(test-rotation "" "" 0)

## Testing the solvers
set(solverAvailable "")

if(LSSOL_FOUND)
    set(use_gfortran ON)
else()
    set(use_gfortran OFF)
endif()

# test the lssol solver
if(LSSOL_FOUND)
  list(APPEND solverAvailable "lssol")
  addUnitTest(test-lssol "" "lssol" ON)
endif(LSSOL_FOUND)


# test the qpoases solver
if(QPOASES_FOUND)
  list(APPEND solverAvailable "QPOASES")
  addUnitTest(test-qpoases "" "QPOASES" OFF)
endif(QPOASES_FOUND)


## Testing all the solvers
addUnitTest(bench-qpsolver   "" "${solverAvailable}" ${use_gfortran})
addUnitTest(test-qpsolver    "" "${solverAvailable}" ${use_gfortran})

## Compare LSSOL and QPOASES.
if(LSSOL_FOUND AND QPOASES_FOUND)
  addUnitTest(test-all-solvers "" "${solverAvailable}" 1)
endif(LSSOL_FOUND AND QPOASES_FOUND)

addUnitTest(test-walkgen  "mpc-walkgen" "" ${use_gfortran})

# a simple execution of the walkgen. For link reasons,
#  it only works with unix systems.
if(UNIX AND NOT APPLE)

  if(LSSOL_FOUND AND QPOASES_FOUND)
    addUnitTest(bench-solvers "mpc-walkgen" "" 1)
  endif(LSSOL_FOUND AND QPOASES_FOUND)

endif(UNIX AND NOT APPLE)
