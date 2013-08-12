////////////////////////////////////////////////////////////////////////////////
///
///\file	qpoases_solver.h
///\brief	Implement the qpoases solver
///\author Lafaye Jory
///\version	1.0
///\date	19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_QPOASES_SOLVER_H
#define MPC_WALKGEN_QPOASES_SOLVER_H

#include "../type.h"
#include <QProblem.hpp>

namespace MPCWalkgen
{
  class QPOasesSolver
  {
  public:
    QPOasesSolver(int nbVar, int nbCtr);

    bool solve(const QPMatrices& m, VectorX& sol,
               bool useWarmStart = false);

    inline int getNbVar() const
    {return nbVar_;}

    inline int getNbCtr() const
    {return nbCtr_;}

  private:
    Eigen::VectorXi constraints_;

    ::qpOASES::QProblem qp_;

    int nbVar_;
    int nbCtr_;

    bool qpIsInitialized_;
  };
}

#endif //MPC_WALKGEN_QPOASES_SOLVER_H
