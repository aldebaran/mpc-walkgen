
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

set (_SOLVER_LIST_ "")
if(LSSOL_FOUND)
  addSolver("LSSOL" _SOLVER_LIST_)
endif(LSSOL_FOUND)

if(QPOASES_FOUND)
  addSolver("QPOASES" _SOLVER_LIST_)
endif(QPOASES_FOUND)

CONFIG_FILES(mpc-walkgen/solvers.h)

