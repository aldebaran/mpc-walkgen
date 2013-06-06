#pragma once
#ifndef MPC_WALKGEN_LSSOL_SOLVER_H
#define MPC_WALKGEN_LSSOL_SOLVER_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	lssol-solver.h
///\brief	A class to solver the QP problem using LSSOL
///\author	Lafaye Jory
///\author      Keith François
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////


#include <Eigen/Dense>
#include "../qp-solver.h"



namespace MPCWalkgen{

  class LSSOLSolver:public QPSolver{
    public:
      LSSOLSolver(const int nbVarMin, const int nbCtrMin, const int nbVarMax, const int nbCtrMax);
      virtual ~LSSOLSolver();

      // accessors
      inline QPSolverType getType() const
      { return QPSOLVERTYPE_LSSOL; }
      inline bool useCholesky() const
      { return useCholesky_; }
      inline void useCholesky(bool /*ch*/)
      { }

      virtual bool solve(Eigen::VectorXd & qpSolution,
             Eigen::VectorXi & constraints,
             Eigen::VectorXd & initialSolution,
             Eigen::VectorXi & initialConstraints,
             Eigen::VectorXd & initialLagrangeMultiplier,
             Eigen::VectorXd & lagrangeMultiplier,
             bool useWarmStart);

    protected:
      virtual bool resizeAll();

    protected:
      Eigen::VectorXi kx_;
      Eigen::VectorXd bb_;
      Eigen::VectorXd bu_;
      Eigen::VectorXd bl_;

      int leniw_;
      int lenw_;
      Eigen::VectorXd war_;
      Eigen::VectorXi iwar_;

      int inform_;
      int iter_;
      double obj_;
      bool useCholesky_;
  };



}

/*!
* \class MPCWalkgen::LSSOLSolver lssol-solver.h "Definition"
* \ingroup solver
* \brief QPSolver based on the lssol library developed by Standford
*/

#endif // MPC_WALKGEN_LSSOL_SOLVER_H
