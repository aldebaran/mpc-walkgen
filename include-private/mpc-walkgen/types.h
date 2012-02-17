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

//TODO: Comment needed
	struct MPCData{
		// The following parameters are fixed once and for all at initialization
		/// \brief Sampling period considered in the QP
		double QPSamplingPeriod;    //blocked - precomputeObjective
		double MPCSamplingPeriod;   //blocked - precomputeObjective / RigidBodySystem::computeDynamicMatrix
		double simSamplingPeriod;   //blocked - precomputeObjective / RigidBodySystem::computeDynamicMatrix

		/// \brief Nb. samplings inside preview window
		int QPNbSamplings;  //blocked - precomputeObjective

		// The following parameters can be changed online
		double stepPeriod;  //blocked by orientPrw_ ? can be solved --
		double DSPeriod;
		double DSSSPeriod;
		int nbStepSSDS;

		/// \brief Compute the unique feedback iteration number between two QP instants
		int iterationNumberFeedback(double firstIterationduration) const;
		/// \brief number of simulation iterations between two feedback call
		int nbIterationSimulation() const;
		/// \brief number of feedback iterations between two QP instants
		int nbIterationFeedback() const;
	};


	struct RobotData{
		//The only parameter changeable online. Watch Walkgen::init
		double CoMHeight;
		double freeFlyingFootMaxHeight;

		//fixed paramters
		FootData leftFoot;
		FootData rightFoot;

		HipYawData leftHipYaw;
		HipYawData rightHipYaw;

		double robotMass;
	};



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

	struct QPPonderation{
		std::vector<double> instantVelocity;
		std::vector<double> CopCentering;
		std::vector<double> JerkMin;

		/// \brief Define the element of ponderation std::vector used in this iteration
		int activePonderation;

		QPPonderation(int nb=1);
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



	struct ConvexHull{
		Eigen::VectorXd x;
		Eigen::VectorXd y;

		Eigen::VectorXd A;
		Eigen::VectorXd B;
		Eigen::VectorXd C;
		Eigen::VectorXd D;

		ConvexHull & operator=(const ConvexHull & hull);
		void resize(int size);
		void rotate(double yaw);
		void computeLinearSystem(const Foot & foot);
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
