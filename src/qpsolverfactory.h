////////////////////////////////////////////////////////////////////////////////
///
///\file	qpsolverfactory.h
///\brief	Templated factory functions for QP solvers
///\author Barthelemy Sebastien
///\version	1.0
///\date	11/12/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_QPSOLVERFACTORY_H
#define MPC_WALKGEN_QPSOLVERFACTORY_H

#include <qi/macro.hpp>
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
template <>
QPSolver<double> *makeQPSolver<double>(int nbVar, int nbCtr);

template <>
QPSolver<float> *makeQPSolver<float>(int nbVar, int nbCtr);
}
#endif //MPC_WALKGEN_QPSOLVERFACTORY_H
