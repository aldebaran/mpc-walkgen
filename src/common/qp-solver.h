#pragma once
#ifndef MPC_WALKGEN_QP_SOLVER_H
#define MPC_WALKGEN_QP_SOLVER_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	qp-solver.h
///\brief	A class to solver the QP problem
///\author	Lafaye Jory
///\author      Keith François
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////


#include <Eigen/Dense>
#include "qp-matrix.h"
#include "qp-vector.h"
#include "types.h"

namespace MPCWalkgen{

  class QPSolver{
    public:
      static const int DefaultNbVarMax_;
      static const int DefaultNbCtrMax_;

    public:
      QPSolver(const int nbVarMin=0, const int nbCtrMin=0, const int nbVarMax=DefaultNbVarMax_, const int nbCtrMax=DefaultNbCtrMax_);
      virtual ~QPSolver()=0;

      void reset();
      virtual bool solve(Eigen::VectorXd & qpSolution,
             Eigen::VectorXi & constraints,
             Eigen::VectorXd & initialSolution,
             Eigen::VectorXi & initialConstraints,
             bool useWarmStart)=0;

      void dump();

    public:
      QPMatrix & matrix(const QPMatrixType type);
      QPVector & vector(const QPVectorType type);

      void nbVar(const int nbVar);
      inline int nbVar() const {return nbVar_;}
      void nbCtr(const int nbCtr);
      inline int nbCtr() const {return nbCtr_;}
      void addNbCtr(const int addCtr);

      void varOrder(const Eigen::VectorXi & order);
      void ctrOrder(const Eigen::VectorXi & order);

      virtual QPSolverType getType() const =0;

      // can we / should we use the cholesly matrix
      virtual bool useCholesky() const =0;
      virtual void useCholesky(bool)=0;

    protected:
      virtual bool resizeAll();

      void reorderInitialSolution(Eigen::VectorXd & initialSolution,
                Eigen::VectorXi & initialConstraints);
      void reorderSolution(Eigen::VectorXd & qpSolution,
               Eigen::VectorXi & constraints,
               Eigen::VectorXi & initialConstraints);

    protected:
      QPMatrix matrixQ_;
      QPMatrix matrixA_;
      QPVector vectorP_;
      QPVector vectorBU_;
      QPVector vectorBL_;
      QPVector vectorXU_;
      QPVector vectorXL_;

      int nbVar_;
      int nbCtr_;
      int nbVarMax_;
      int nbCtrMax_;

      Eigen::VectorXi varOrder_;
      Eigen::VectorXi ctrOrder_;
  };
  QPSolver* createQPSolver(QPSolverType solvertype,
      int nbVarMin=0,
      int nbCtrMin=0,
      int nbVarMax=QPSolver::DefaultNbVarMax_,
      int nbCtrMax=QPSolver::DefaultNbCtrMax_);
}


/*! @defgroup solver solvers
 *  @ingroup private
 * this group gathers the solver available with mpc-walkgen
 */

/*!
* \class MPCWalkgen::QPSolver qp-solver.h "Definition"
* \ingroup solver
* \brief Abstract interface of the solver used
*/

/*! \fn MPCWalkgen::QPSolver::QPSolver(const int nbVarMax=DefaultNbVarMax_, const int nbCtrMax=DefaultNbCtrMax_)
* \brief Constructor
* \param nbVarMax Maximum number of variables
* \param nbCtrMax Maximum number of constraints
*/

/*! \fn QPMatrix & MPCWalkgen::QPSolver::matrix(const QPMatrixType type)
* \brief Return the desired QP matrix
*/

/*! \fn MPCWalkgen::QPSolver::matrix(const QPMatrixType type);
* \brief return a QPMatrix.
*/

/*! \fn MPCWalkgen::QPSolver::reset(const bool withConstantPart = true)
* \brief Erase the problem
* \param withConstantPart if true, constant part of QPMatrices will replace current values
*/

/*! \fn MPCWalkgen::QPSolver::nbVar(const int nbVar)
* \brief Setter to modify the number of variables
*/

/*! \fn MPCWalkgen::QPSolver::nbCtr(const int nbCtr)
* \brief Setter to modify the number of constraints
*/

/*! \fn MPCWalkgen::QPSolver::addNbCtr(const int addCtr)
* \brief Augment the number of constraints
*/

/*! \fn Solver MPCWalkgen::QPSolver::getType() const
* \brief Return the type of solver
*/

/*! \fn bool MPCWalkgen::QPSolver::useCholesky() const
* \brief Return true is the solver makes use of the cholesky matrix
*/

/*! \fn void MPCWalkgen::QPSolver::useCholesky(bool)
* \brief indicates if the solver can use of the cholesky matrix (unavailable for some solvers)
*/

/*! \fn MPCWalkgen::QPSolver::addNbCtr(const int addCtr)
* \brief Augment the number of constraints
*/

/*! \fn MPCWalkgen::QPSolver::solve(MPCSolution & solution)
* \brief Solve the problem defined by this form : \f$ \left\{
* \begin{array}{l}
* 	min \| x^t Q x + p^t x \|\\
* 	bl \leq Ax \leq bu, \\
* 	xl \leq x \leq xu \\
* \end{array}
* \right.
* \f$
*/
#endif // MPC_WALKGEN_QP_SOLVER_H
