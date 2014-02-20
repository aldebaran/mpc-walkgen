////////////////////////////////////////////////////////////////////////////////
///
///\file qpsolverfactory.h
///\brief Templated factory functions for QP solvers
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_QPSOLVERFACTORY_H
#define MPC_WALKGEN_QPSOLVERFACTORY_H

#include <mpc-walkgen/api.h>
#include <mpc-walkgen/qpsolver.h>

namespace MPCWalkgen
{

template <typename Scalar>
QPSolver<Scalar> *makeQPSolver(int nbVar, int nbCtr)
{
  // useless default implementation
  std::abort();
  return NULL;
}

// useful specializations
template <> MPC_WALKGEN_API
QPSolver<double> *makeQPSolver<double>(int nbVar, int nbCtr);

template <> MPC_WALKGEN_API
QPSolver<float> *makeQPSolver<float>(int nbVar, int nbCtr);
}
#endif
