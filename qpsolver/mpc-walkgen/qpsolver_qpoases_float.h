////////////////////////////////////////////////////////////////////////////////
///
///\file	qpsolver_qpoases_double.h
///\brief	Factory for qpoases solver, compiled with double
///\author Barthelemy Sebastien
///\version	1.0
///\date	10/12/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_QPSOLVER_QPOASES_FLOAT_H
#define MPC_WALKGEN_QPSOLVER_QPOASES_FLOAT_H
#include <qi/macro.hpp>
#define MPC_WALKGEN_QPSOLVER_QPOASES_FLOAT_API QI_LIB_API(mpc_walkgen_qpsolver_qpoases_float)
#include <mpc-walkgen/qpsolver.h>

namespace MPCWalkgen
{
  // factory: return a qpOASES solver
  MPC_WALKGEN_QPSOLVER_QPOASES_FLOAT_API
  QPSolver<float> *makeQPSolverFloat(int nbVar, int nbCtr);
}

#endif //MPC_WALKGEN_QPSOLVER_QPOASES_FLOAT_H
