#ifndef QP_SOLVER
#define QP_SOLVER

////////////////////////////////////////////////////////////////////////////////
///
///\file	qp-solver.h
///\brief	A class to solver the QP problem
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author	Keith François
///\version	1.0
///\date	05/01/12
///
////////////////////////////////////////////////////////////////////////////////


#include <Eigen/Dense>
#include <mpc-walkgen/qp-matrix.h>
#include <mpc-walkgen/qp-vector.h>
#include <mpc-walkgen/types.h>


namespace MPCWalkgen{

	class QPSolver{
		public:
			static const int DefaultNbVarMax_;
			static const int DefaultNbCtrMax_;

		public:
			QPSolver(const int nbVarMin=0, const int nbCtrMin=0, const int nbVarMax=DefaultNbVarMax_, const int nbCtrMax=DefaultNbCtrMax_);
			virtual ~QPSolver()=0;

			void reset();
			virtual void solve(MPCSolution & result)=0;

			void dump();

		public:
			QPMatrix & matrix(const QPMatrixType type);
			QPVector & vector(const QPMatrixType type);

			void nbVar(const int nbVar);
			void nbCtr(const int nbCtr);
			inline int nbCtr() const {return nbCtr_;}
			void addNbCtr(const int addCtr);

			void varOrder(const Eigen::VectorXi & order);
			void ctrOrder(const Eigen::VectorXi & order);

			virtual Solver getType() const =0;

			// can we / should we use the cholesly matrix
			virtual bool useCholesky() const =0;
			virtual void useCholesky(bool)=0;

		protected:
			virtual bool resizeAll();

			void reorderInitialSolution(MPCSolution & solution);
			void reorderSolution(MPCSolution & solution);

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
#endif //QP_SOLVER
