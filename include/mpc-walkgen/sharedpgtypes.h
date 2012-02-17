/*
 * Copyright 2010,
 *
 * Andrei Herdt
 * Olivier  Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
/*! \file sharedpgtypes.h
  \brief Defines basic types for the Humanoid Walking Pattern Generator.
*/

#ifndef _PATTERN_GENERATOR_INTERNAL_SHARED_H_2
#define  _PATTERN_GENERATOR_INTERNAL_SHARED_H_2

#include <deque>
#include <Eigen/Dense>
#include <vector>


// For Windows compatibility.
#if defined (WIN32)
#  ifdef mpc_walkgen_EXPORTS
#    define MPC_WALK_GEN_EXPORT __declspec(dllexport)
#  else
#    define MPC_WALK_GEN_EXPORT __declspec(dllimport)
#  endif
#else
#  define MPC_WALK_GEN_EXPORT
#endif

namespace MPCWalkgen
{
	//
	// Enum types
	//

	/// \name Enum types
	/// \{
	enum Phase{
		SS,
		DS
	};

	enum Foot{
		LEFT,
		RIGHT
	};

	enum BodyType{
		LEFT_FOOT,
		RIGHT_FOOT,
		COM
	};
	/// \}

	//
	// Structures
	//

	/// \name Structures
	/// \{
	struct FootData{
		double soleWidth_;
		double soleHeight_;
		Eigen::Vector3d anklePositionInLocalFrame_;

		inline FootData()
		  : soleWidth_(0)
		  , soleHeight_(0)
		  , anklePositionInLocalFrame_()
		  {}

		inline FootData(const FootData &f)
		  : soleWidth_(f.soleWidth_)
		  , soleHeight_(f.soleHeight_)
		  , anklePositionInLocalFrame_(f.anklePositionInLocalFrame_)//TODO: LocalAnklePosition_ better?
		  {}

	};

	struct HipYawData{
		double lowerBound_;
		double upperBound_;
		double lowerVelocityBound_;
		double upperVelocityBound_;
		double lowerAccelerationBound_;
		double upperAccelerationBound_;

		MPC_WALK_GEN_EXPORT HipYawData(
				double lowerBound =  0				,   double upperBound = 0,
				double lowerVelocityBound =  -3.54108,	double upperVelocityBound = 3.54108,
				double lowerAccelerationBound = -0.1,	double upperAccelerationBound = 0.1
		);
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

	struct QPPonderation {
		std::vector<double> instantVelocity;
		std::vector<double> CopCentering;
		std::vector<double> JerkMin;

		/// \brief Define the element of ponderation std::vector used in this iteration
		int activePonderation;

		QPPonderation(int nb = 2);
	};

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

		QPPonderation ponderation;

		MPCData();
	};

	struct RobotData{
		double CoMHeight;
		double freeFlyingFootMaxHeight;

  		FootData leftFoot;
  		FootData rightFoot;

  		HipYawData leftHipYaw;
  		HipYawData rightHipYaw;

  		double robotMass;
  	};

	struct SupportState{
		Phase phase;
		Foot foot;

		int nbStepsLeft;
		int stepNumber;
		int nbInstants;

		double timeLimit;
		double startTime;

		double x,y,yaw;
		double yawTrunk;//TODO: Why in SupportState? -> for compatibility with temporary previewROrientation class

		bool stateChanged;

		/// \brief Define if the support state is in double support between two single support
		bool inTransitionPhase;

		/// \brief The duration of the support state
		double iterationDuration;

		// \brief The relative weight of this support state in the QP (A support state duration of QPSamplingTime have : iterationWeight = 1)
		double iterationWeight;
	};

	struct BodyState{
		Eigen::Vector3d x;
		Eigen::Vector3d y;
		Eigen::Vector3d z;
		Eigen::Vector3d yaw;

		BodyState();

		void reset();
	};

	struct MPCSolution{

		MPC_WALK_GEN_EXPORT MPCSolution();

		MPC_WALK_GEN_EXPORT void reset();

		// attributes
		Eigen::VectorXd solution;
		Eigen::VectorXd initialSolution;

		Eigen::VectorXi constraints;
		Eigen::VectorXi initialConstraints;

		bool useWarmStart;
		/// \brief True if a new trajectory is computed in online loop
		bool newTraj;

		std::vector<SupportState> supportState_vec;
		SupportState currentSupportState;

		std::vector<double> supportOrientation_vec;
		std::vector<double> supportTrunkOrientation_vec;

		Eigen::VectorXd CoPTrajX;
		Eigen::VectorXd CoPTrajY;

		//TODO: Why does State contain trajectory vectors?
		//TODO: State is used in a vector of states
		struct State
		{
			Eigen::VectorXd CoMTrajX_;
			Eigen::VectorXd CoMTrajY_;

			Eigen::VectorXd leftFootTrajX_;
			Eigen::VectorXd leftFootTrajY_;
			Eigen::VectorXd leftFootTrajZ_;
			Eigen::VectorXd leftFootTrajYaw_;

			Eigen::VectorXd rightFootTrajX_;
			Eigen::VectorXd rightFootTrajY_;
			Eigen::VectorXd rightFootTrajZ_;
			Eigen::VectorXd rightFootTrajYaw_;

			Eigen::VectorXd trunkYaw_;
		};
		std::vector<State> state_vec;
	};
}

#endif /* _PATTERN_GENERATOR_INTERNAL_PRIVATE_H_ */
