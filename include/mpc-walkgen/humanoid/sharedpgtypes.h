#pragma once
#ifndef MPC_WALKGEN_HUMANOID_SHAREDPGTYPE_H
#define  MPC_WALKGEN_HUMANOID_SHAREDPGTYPE_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	api.h
///\brief	Definition of humanoid types
///\author	Lafaye Jory
///\author      Keith François
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/common/sharedpgtypes.h>
#include <Eigen/Dense>
#include <vector>





namespace MPCWalkgen{
  namespace Humanoid{
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



    struct MPC_WALKGEN_API FootData{
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

    struct MPC_WALKGEN_API HipYawData{
      double lowerBound;
      double upperBound;
      double lowerVelocityBound;
      double upperVelocityBound;
      double lowerAccelerationBound;
      double upperAccelerationBound;

      HipYawData();
    };

    struct MPC_WALKGEN_API QPPonderation{
      std::vector<double> instantVelocity;
      std::vector<double> CopCentering;
      std::vector<double> JerkMin;

      /// \brief Define the element of ponderation std::vector used in this iteration
      int activePonderation;

      QPPonderation(int nb = 2);
    };

    struct MPC_WALKGEN_API SupportState{
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

    struct MPC_WALKGEN_API ConvexHull {
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

    struct MPC_WALKGEN_API MPCData{
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

    struct MPC_WALKGEN_API RobotData {
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

    struct MPC_WALKGEN_API MPCSolution{

      MPCSolution();

      void reset();

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
}

#endif // MPC_WALKGEN_HUMANOID_SHAREDPGTYPE_H
