
#pragma once
#ifndef MPC_WALKGEN_QPOASES_SOLVER_H
#define MPC_WALKGEN_QPOASES_SOLVER_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	qpoases-solver.h
///\brief	A class to solver the QP problem using qpoases solver
///\author	Lafaye Jory
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////


#include <Eigen/Dense>
#include "../qp-solver.h"
#include <vector>


#ifdef USE_QPOASES_3_0
# include <qpOASES/QProblem.hpp>
#else //
# include <QProblem.hpp>
#endif //USE_QPOASES_3_0


namespace MPCWalkgen{



  class QPOasesSolver:public QPSolver{
    public:
      QPOasesSolver(const int nbVarMin, const int nbCtrMin, const int nbVarMax, const int nbCtrMax);
      virtual ~QPOasesSolver();

      // accessors
      inline QPSolverType getType() const
      { return QPSOLVERTYPE_QPOASES; }
      inline bool useCholesky() const
      { return false; }
      inline void useCholesky(bool /*ch*/)
      {}

      virtual bool solve(Eigen::VectorXd & qpSolution,
             Eigen::VectorXi & constraints,
             Eigen::VectorXd & initialSolution,
             Eigen::VectorXi & initialConstraints,
             bool useWarmStart);

    protected:
      virtual bool resizeAll();

    protected:
      std::vector< ::qpOASES::QProblem*> qp_;
      std::vector< ::qpOASES::Constraints*> ctrInit_;
      std::vector< ::qpOASES::Bounds*> boundsInit_;
      int nbVarMin_;
      int nbCtrMin_;
      int sizeNbCtr_;
      int sizeNbVar_;


  };



}

/*!
* \class MPCWalkgen::QPOasesLSolver qpoases-solver.h "Definition"
* \ingroup solver
* \brief QPSolver based on the qpoases library
*/

#endif // MPC_WALKGEN_QPOASES_SOLVER_H
