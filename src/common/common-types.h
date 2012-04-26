#ifndef PGTYPECOMMON
#define PGTYPECOMMON

////////////////////////////////////////////////////////////////////////////////
///
///\file	common-types.h
///\brief	Definition of types used in MPC
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author	Keith François
///\version	1.0
///\date	05/01/12
///
////////////////////////////////////////////////////////////////////////////////

#include <deque>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <mpc-walkgen/sharedpgtypes.h>


namespace MPCWalkgen{

	static const double EPSILON = 0.000001;

	//TODO: The QP matrices and vectors in the class solver can be encapsulated
	// QPProblem & QP = solver_.QP();
	// QP.vectoP.addTerm();
	// instead of solver_->matrix(VECTOR_P).addTerm();
	// which would also make these enums unnecessary
	enum QPMatrixType{
		matrixQ,
		matrixA,
		vectorP,
		vectorBU,
		vectorBL,
		vectorXU,
		vectorXL
	};


	struct LinearDynamics{
		Eigen::MatrixXd S;
		Eigen::MatrixXd U;
		Eigen::MatrixXd UT;
		Eigen::MatrixXd UInv;
		Eigen::MatrixXd UInvT;
	};
}


#endif //PGTYPECOMMON
