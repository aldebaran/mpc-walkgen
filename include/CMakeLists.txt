
# addSolver:  create the list of solvers available that will be used
#             to create a enum list in C++
# solver:     the name of the enum.
# solverList: the enum list
macro(addSolver solver solverList)
  if(NOT ${solverList})
    SET(${solverList} "\t\t${solver}")
  else(NOT solverList)
    SET(${solverList} "${${solverList}},\n\t\t${solver}")
  endif(NOT ${solverList})
endmacro(addSolver)


find_package("QPOASES")
find_package("LSSOL")

message(STATUS "LSSOL_FOUND: ${LSSOL_FOUND}")

set (_SOLVER_LIST_ "")
if(LSSOL_FOUND)
  addSolver("LSSOL" _SOLVER_LIST_)
endif(LSSOL_FOUND)

if(QPOASES_FOUND)
  addSolver("QPOASES" _SOLVER_LIST_)
endif(QPOASES_FOUND)

CONFIGURE_FILE(mpc-walkgen/solvers.h.in mpc-walkgen/solvers.h)

