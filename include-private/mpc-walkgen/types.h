#ifndef PGTYPE
#define PGTYPE

////////////////////////////////////////////////////////////////////////////////
///
///\file	types.h
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

#include <mpc-walkgen/enums.h>
#include <mpc-walkgen/sharedpgtypes.h>

#define EPS 0.0000001

namespace MPCWalkgen{

	struct VelReference{
		struct Frame{
			double x;
			double y;
			double yaw;

			Eigen::VectorXd xVec;
			Eigen::VectorXd yVec;

			Frame();
		};

		Frame global;
		Frame local;

		VelReference();
	};

	struct SelectionMatrices{
		Eigen::MatrixXd V;
		Eigen::MatrixXd VT;
		Eigen::VectorXd VcX;
		Eigen::VectorXd VcY;
		Eigen::MatrixXd Vf;
		Eigen::VectorXd VcfX;
		Eigen::VectorXd VcfY;

		SelectionMatrices(const MPCData & generalData);
	};

	struct DynamicMatrix{
		Eigen::MatrixXd S;
		Eigen::MatrixXd U;
		Eigen::MatrixXd UT;
		Eigen::MatrixXd UInv;
		Eigen::MatrixXd UInvT;
	};

	struct RelativeInequalities{
		Eigen::MatrixXd DX;
		Eigen::MatrixXd DY;
		Eigen::VectorXd Dc;

		void resize(int rows, int cols);
	};



}

/** @defgroup private MPCWalkgen private interface
 *  This group gathers the classes contained in the private interface
 */


#endif //PGTYPE
