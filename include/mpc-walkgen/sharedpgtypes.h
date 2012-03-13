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

        enum Solver{
                QPOASES,
                LSSOL
        };

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
		double soleWidth;
		double soleHeight;
		Eigen::Vector3d anklePositionInLocalFrame;

		inline FootData()
		  : soleWidth(0)
		  , soleHeight(0)
		  , anklePositionInLocalFrame()
		  {}

		inline FootData(const FootData &f)
		  : soleWidth(f.soleWidth)
		  , soleHeight(f.soleHeight)
		  , anklePositionInLocalFrame(f.anklePositionInLocalFrame)//TODO: LocalAnklePosition_ better?
		  {}

	};

	struct HipYawData{
		double lowerBound;
		double upperBound;
		double lowerVelocityBound;
		double upperVelocityBound;
		double lowerAccelerationBound;
		double upperAccelerationBound;

		HipYawData();
	};

	struct QPPonderation {
		std::vector<double> instantVelocity;
		std::vector<double> CopCentering;
		std::vector<double> JerkMin;

		/// \brief Define the element of ponderation std::vector used in this iteration
		int activePonderation;

		QPPonderation(int nb = 2);
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

		/// \brief Define if the support state is in a (transitional) double support phase
		bool inTransitionalDS;

		/// \brief The length of the previous sampling period (can be different from QPSamplingPeriod)
		double previousSamplingPeriod;//TODO: change name

		// \brief The relative weight of this support state in the QP (A support state duration of QPSamplingTime have : iterationWeight = 1)
		double sampleWeight;//TODO: shouldn't it be outside... somewhere...?
	};

	struct ConvexHull {
		/// \brief Set of vertices
		Eigen::VectorXd x;
		Eigen::VectorXd y;
		Eigen::VectorXd z;

		/// \brief Set of inequalities A*x+B*y+C*z+D>0
		Eigen::VectorXd A;
		Eigen::VectorXd B;
		Eigen::VectorXd C;
		Eigen::VectorXd D;

		ConvexHull();
		ConvexHull &operator=(const ConvexHull &hull); // TODO: copyFrom() instead of =
		void resize(int size);
		void rotate(double yaw);
		void computeLinearSystem(const Foot &foot);
	};

	struct MPCData{
		// The following parameters are fixed once and for all at initialization
		/// \brief Sampling period considered in the QP
		double QPSamplingPeriod;    //blocked - precomputeObjective
		double MPCSamplingPeriod;   //blocked - precomputeObjective / RigidBodySystem::computeDynamicMatrix
		double actuationSamplingPeriod;   //blocked - precomputeObjective / RigidBodySystem::computeDynamicMatrix

		/// \brief Nb. samplings inside preview window
		int nbSamplesQP;  //blocked - precomputeObjective

		// The following parameters can be changed online
		double stepPeriod;  //blocked by orientPrw_ ? can be solved --
		double DSPeriod;
		double DSSSPeriod;
		int nbStepSSDS;

		/// \brief Compute the number of recomputations left until next sample
		int nbFeedbackSamplesLeft(double firstSamplingPeriod) const;
		/// \brief number of simulation iterations between two feedback call
		int nbSamplesControl() const;
		/// \brief number of feedback iterations between two QP instants
		int nbFeedbackSamplesStandard() const;

		QPPonderation ponderation;

		MPCData();
	};

	struct RobotData {
		double CoMHeight;
		double freeFlyingFootMaxHeight;

  		FootData leftFoot;
  		FootData rightFoot;

  		HipYawData leftHipYaw;
  		HipYawData rightHipYaw;

  		double robotMass; //TODO: rename mass

  		Eigen::Vector3d leftFootPos;
  		Eigen::Vector3d rightFootPos;

		ConvexHull leftFootHull;
		ConvexHull rightFootHull;
		ConvexHull CoPLeftSSHull;
		ConvexHull CoPRightSSHull;
		ConvexHull CoPLeftDSHull;
		ConvexHull CoPRightDSHull;

  		RobotData(const FootData &leftFoot, const FootData &rightFoot,
  				const HipYawData &leftHipYaw, const HipYawData &rightHipYaw,
  				double mass);
  		RobotData();
  	};

	struct BodyState{
		Eigen::Vector3d x;
		Eigen::Vector3d y;
		Eigen::Vector3d z;
		Eigen::Vector3d yaw;
		Eigen::Vector3d pitch;
		Eigen::Vector3d roll;

		BodyState();

		void reset();
	};

	struct MPCSolution{

		MPC_WALK_GEN_EXPORT MPCSolution();

		MPC_WALK_GEN_EXPORT void reset();

		// attributes
		Eigen::VectorXd qpSolution;
		Eigen::VectorXd initialSolution;

		Eigen::VectorXi constraints;
		Eigen::VectorXi initialConstraints;

		bool useWarmStart;
		/// \brief True if a new trajectory is computed in online loop
		bool newTraj;

		/// \brief Sampling times
		/// starting with 0, i.e. all times are relative to the current time
		std::vector<double> samplingTimes_vec;

		std::vector<SupportState> supportStates_vec;

		std::vector<double> supportOrientations_vec;//TODO: supportOrientations_vec
		std::vector<double> supportTrunkOrientations_vec;//TODO: TrunkOrientations_vec

		Eigen::VectorXd CoPTrajX;
		Eigen::VectorXd CoPTrajY;

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
