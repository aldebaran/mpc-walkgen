#pragma once
#ifndef MPC_WALKGEN_ZEBULON_SHAREDPGTYPE_H
#define  MPC_WALKGEN_ZEBULON_SHAREDPGTYPE_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	api.h
///\brief	Definition of zebulon types
///\author	Lafaye Jory
///\author      Keith François
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/common/sharedpgtypes.h>
#include <Eigen/Dense>
#include <vector>

// Disable Warning CS4251: need to have dll-interface
#ifdef _MSC_VER
 #pragma warning( push )
 #pragma warning( disable: 4251 )
#endif

namespace MPCWalkgen{
  namespace Zebulon{
    //
    // Enum types
    //

    /// \name Enum types
    /// \{

    enum BodyType{
      BASE,
      COM
    };

    /// \}

    //
    // Structures
    //

    /// \name Structures
    /// \{



    struct MPC_WALKGEN_API QPPonderation{
      std::vector<double> baseInstantVelocity;
      std::vector<double> basePosition;
      std::vector<double> basePositionInt;
      std::vector<double> CopCentering;
      std::vector<double> CoMCentering;
      std::vector<double> CoMJerkMin;
      std::vector<double> baseJerkMin;
      std::vector<double> OrientationInstantVelocity;
      std::vector<double> OrientationPosition;
      std::vector<double> OrientationJerkMin;

      /// \brief Define the element of ponderation std::vector used in this iteration
      int activePonderation;

      QPPonderation(int nb = 4);
    };

    struct MPC_WALKGEN_API MPCData{
      // The following parameters are fixed once and for all at initialization
      /// \brief Sampling period considered in the QP
      double QPSamplingPeriod;    //blocked - precomputeObjective
      double MPCSamplingPeriod;   //blocked - precomputeObjective / RigidBodySystem::computeDynamicMatrix
      double actuationSamplingPeriod;   //blocked - precomputeObjective / RigidBodySystem::computeDynamicMatrix

      /// \brief Nb. samplings inside preview window
      int nbSamplesQP;  //blocked - precomputeObjective

      /// \brief number of simulation iterations between two feedback call
      int nbSamplesControl() const;
      int nbSamplesMPC() const;
      QPPonderation ponderation;


      int QPNbVariables; //Number of variables in the QP
      int QPNbConstraints; //Number of constraints in the QP
      int QPOrientationNbVariables; //Number of variables in the orientation QP
      int QPOrientationNbConstraints; //Number of constraints in the orientation QP

      MPCData();
    };

    struct MPC_WALKGEN_API RobotData {
      double CoMHeight;

      double copLimitX;
      double copLimitY;
      double deltaComXLocal;
      std::vector<double> baseLimit;
      std::vector<double> orientationLimit;
      double comLimitX;
      double comLimitY;
      Eigen::Vector3d gravity;

      RobotData();
    };

    //Environment information :
    // * obstacle{PositionX, PositionY, Radius} are vectors of size nbObstacle
    //   which represent the position and the radius of nbObstacle obstacles of circular shape.
    //
    // * obstacleLinearizationPoint{X, Y} are vectors of size MPCData.nbSamplesQP
    //   which represent the linearization points in the preview
    //   to convexify the obstacles.
    //
    //note : nbObstacle <= nbObstacleMax
    struct MPC_WALKGEN_API EnvData {
      static int nbObstacleMax;

      EnvData()
      :nbObstacle(0)
      {}

      Eigen::VectorXd obstaclePositionX;
      Eigen::VectorXd obstaclePositionY;
      Eigen::VectorXd obstacleRadius;
      Eigen::VectorXd obstacleLinearizationPointX;
      Eigen::VectorXd obstacleLinearizationPointY;

      double nbObstacle;
    };



    struct MPC_WALKGEN_API MPCSolution{

      MPCSolution();

      /// \brief True if a new trajectory is computed in online loop
      bool newTraj;

      Eigen::VectorXd CoPTrajX;
      Eigen::VectorXd CoPTrajY;

      struct State
      {
        Eigen::VectorXd CoMTrajX_;
        Eigen::VectorXd CoMTrajY_;
        Eigen::VectorXd CoMTrajYaw_;
        Eigen::VectorXd baseTrajX_;
        Eigen::VectorXd baseTrajY_;

        State();

      };
      std::vector<State> state_vec;

      bool solutionFound;
    };
  }
}

#ifdef _MSC_VER
#  pragma warning( pop )
#endif

#endif // MPC_WALKGEN_ZEBULON_SHAREDPGTYPE_H
