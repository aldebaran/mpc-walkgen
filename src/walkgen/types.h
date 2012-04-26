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

#include <mpc-walkgen/sharedpgtypes-humanoid.h>
#include "../common/common-types.h"

namespace MPCWalkgen{


	enum HullType{
		FootHull,
		CoPHull
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
