#ifndef LSSOL_SOLVER
#define LSSOL_SOLVER

////////////////////////////////////////////////////////////////////////////////
///
///\file	lssol-solver.h
///\brief	A class to solver the QP problem using LSSOL
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author	Keith François
///\version	1.0
///\date	05/01/12
///
////////////////////////////////////////////////////////////////////////////////


#include <Eigen/Dense>
#include <mpc-walkgen/qp-solver.h>
#include <mpc-walkgen/types.h>


namespace MPCWalkgen{

	class LSSOLSolver:public QPSolver{
		public:
			LSSOLSolver(const int nbVarMax=QPSolver::DefaultNbVarMax_, const int nbCtrMax=QPSolver::DefaultNbCtrMax_);
			virtual ~LSSOLSolver();

			// accessors
			inline bool useCholesky()
			{ return useCholesky_; }
			inline void useCholesky(bool /*ch*/)
			{ }

			virtual void solve(MPCSolution & solution);

		protected:
			virtual bool resizeAll();

		protected:
			Eigen::VectorXi kx_;
			Eigen::VectorXd bb_;
			Eigen::VectorXd lambda_;
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

#endif //LSSOL_SOLVER
