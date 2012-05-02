#ifndef PGTYPECOMMON
#define PGTYPECOMMON

////////////////////////////////////////////////////////////////////////////////
///
///\file	common-types.h
///\brief	Definition of types used in MPC
///\author	Lafaye Jory
///\author      Keith François
///\author	Herdt Andrei
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////


#include <Eigen/Dense>

#include <iostream>

#include <mpc-walkgen/sharedpgtypes-common.h>

namespace MPCWalkgen{

	static const double EPSILON = 0.000001;

	enum QPMatrixType{
		matrixQ,
		matrixA
	};

	enum QPVectorType{
		vectorP,
		vectorBU,
		vectorBL,
		vectorXU,
		vectorXL
	};

	enum DynamicMatrixType{
		posDynamic,
		velDynamic,
		accDynamic,
		jerkDynamic,
		copDynamic,
		interpolationPos,
		interpolationVel,
		interpolationAcc,
		interpolationCoP
	};

	struct LinearDynamics{
		Eigen::MatrixXd S;
		Eigen::MatrixXd U;
		Eigen::MatrixXd UT;
		Eigen::MatrixXd UInv;
		Eigen::MatrixXd UInvT;
	};

	struct VelReference{
		struct Frame{
			Eigen::VectorXd x;
			Eigen::VectorXd y;
			Eigen::VectorXd yaw;

			Frame();

			void resize(int size);
		};

		Frame global;
		Frame local;

		VelReference();

		void resize(int size);
	};

}


#endif //PGTYPECOMMON
