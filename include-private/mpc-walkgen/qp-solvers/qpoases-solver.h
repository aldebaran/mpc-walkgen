#ifndef QPOASES_SOLVER
#define QPOASES_SOLVER

////////////////////////////////////////////////////////////////////////////////
///
///\file	qpoases-solver.h
///\brief	A class to solver the QP problem using qpoases solver
///\author	Lafaye Jory
///\version	1.0
///\date	05/01/12
///
////////////////////////////////////////////////////////////////////////////////


#include <Eigen/Dense>
#include <mpc-walkgen/qp-solver.h>
#include <mpc-walkgen/types.h>

namespace qpOASES{
  class QProblem;
}
namespace MPCWalkgen{



	class QPOasesSolver:public QPSolver{
		public:
			QPOasesSolver(const int nbVarMax=QPSolver::DefaultNbVarMax_, const int nbCtrMax=QPSolver::DefaultNbCtrMax_);
			virtual ~QPOasesSolver();

			// accessors
			inline Solver getType() const
			{ return QPOASES; }
			inline bool useCholesky() const
			{ return false; }
			inline void useCholesky(bool /*ch*/)
			{}

			virtual void solve(MPCSolution & solution);

		protected:
			virtual bool resizeAll();

		protected:
			::qpOASES::QProblem* qp_;


	};



}

/*!
* \class MPCWalkgen::QPOasesLSolver qpoases-solver.h "Definition"
* \ingroup solver
* \brief QPSolver based on the qpoases library
*/

#endif //QPOASES_SOLVER
