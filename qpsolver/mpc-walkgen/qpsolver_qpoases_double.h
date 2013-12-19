////////////////////////////////////////////////////////////////////////////////
///
///\file qpsolver_qpoases_double.h
///\brief Factory for qpoases solver, compiled with double
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_QPSOLVER_QPOASES_DOUBLE_H
#define MPC_WALKGEN_QPSOLVER_QPOASES_DOUBLE_H
#include <qi/macro.hpp>
#define MPC_WALKGEN_QPSOLVER_QPOASES_DOUBLE_API QI_LIB_API(mpc_walkgen_qpsolver_qpoases_double)
#include <mpc-walkgen/qpsolver.h>

namespace MPCWalkgen
{
  // factory: return a qpOASES solver
  MPC_WALKGEN_QPSOLVER_QPOASES_DOUBLE_API
  QPSolver<double> *makeQPSolverDouble(int nbVar, int nbCtr);
}

#endif
