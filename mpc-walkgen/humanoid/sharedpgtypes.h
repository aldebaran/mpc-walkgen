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

#ifdef _MSC_VER
# pragma warning( push )
// disable warning C4251: need to have DLL interface
# pragma warning( disable: 4251 )
#endif

//-------------------------------//READ CAREFULLY BEFORE USE//----------------------------------//
/* MPC Walkgen uses three different sampling periods, three different frames and it also considers
  different phases while the robot is walking. In order to avoid confusing situation, please read
  the three items below.


// Sampling times //
 There are three different sampling periods in MPC-Walkgen:

 --The QP sampling period (QPSamplingPeriod). It cannot be chosen freely
   (see "Phases during walking")
 --The MPC sampling period (MPCSamplingPeriod), which corresponds to the time
   between each call of the MPC.
 --The actuation sampling period (actuationSamplingPeriod), which is the period between each instant
   we send samples to the actuators. It allows to generate several interpolated samples for each
   call of the MPC.

 And we have: QPSamplingPeriod >= MPCSamplingPeriod >= actuationSamplingPeriod
 Also, we have QPSamplingPeriod = n * MPCSamplingPeriod = n * m *actuationSamplingPeriod,
 where n and m are integers



//Phases during walking//

 While walking, the robot goes through a movement phase where one of its feet is on the ground
 while the other one is moving. We will call this phase "Simple Support", or "SS".

 Between two steps, both feet are on the ground. We will call this phase
 "Transitional Double Support". However, no QP sample can fall within a Transitional DS.
 Hence QPSamplingPeriod must be equal to the Transitional DS phase duration. This also means that
 during walking, the robot stays continuously in a SS phase until it stops. Note that during a
 DS phase, the support foot is the left one

 On the very beginning and the very end of the walking process, both feet are also on the ground.
 But this time, some QP samples falls within this phase. We will call it "Double Support", or "DS".
 Also, we may use in comments the terms "first" or "final" to refer to the DS support phase on the
 beginning of the walking process or the DS support phase on the end of the walking process
 respectively.



//Frames//

 --The world frame is a fixed Galilean frame attached to the ground.
 --The local frame relative to the current step. It is a fixed Galilean frame which origin and
   orientation are the position and the orientation of the current support foot. We will call this
   frame "Current local frame"
 --The local frame relative to each planned step. It is a piecewise (along time) fixed Galilean
   frame. Its origin and orientation are the position and the orientation of the current support
   foot for a given time. If you are working with future information, e.g. the CoP position during
   the second planned step, then this frame will be attached to the second support foot. We will
   call this frame "Time-dependent local frame"

   Note that the position of each foot is defined as the origin of the frame in which
   CoP hull vertices coordinates are given.

*/

namespace MPCWalkgen{
  namespace Humanoid{
    //
    // Enum types
    //

    /// \name Enum types
    /// \{
    enum Phase{
      SS,
      DS /* WARNING: Double support refers to the phases on the very beginning and on the
            very end of the walking. Phase will never be DS during an actual walking process */
    };

    enum Foot{
      LEFT,
      RIGHT
    };

    /*The COM is here defines by its position and the robot torso orientation*/
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

    struct MPC_WALKGEN_API HipYawData{
      /* Values of lower and upper limits of the hip yaw angle, its derivative and second derivative
         Note that lowerVelocityBound and lowerAccelerationBound are not used across MPC-Walkgen */
      double lowerBound;
      double upperBound;
      double lowerVelocityBound;
      double upperVelocityBound;
      double lowerAccelerationBound;
      double upperAccelerationBound;

      HipYawData();
    };


    /* Weighting of the QP problem cost function
     We use vectors so that we can choose different sets of weights for different cases of use */
    struct MPC_WALKGEN_API QPWeighting{
      std::vector<double> instantVelocity; // Instant velocity of the CoM along X and Y axis
      std::vector<double> CopCentering; // CoP centering on support foot position
      std::vector<double> JerkMin; // Jerk of the CoM along X and Y axis

      /// \brief Define the element of weighting std::vector used in this iteration
      int activeWeighting;

      QPWeighting(int nb = 2);
    };

    // Provides the needed information for a support state
    struct MPC_WALKGEN_API SupportState{
      Phase phase; //DS or SS
      Foot foot;   //LEFT or RIGHT

      int nbStepsLeft;// Number of step remaining after this support state.
      int stepNumber; /* Index of this support state in the vector of support states
                         planned by the MPC*/
      int nbInstants; // Number of QP samples since the last support state change

      double timeLimit; // Absolute time of the end of this support state
      double startTime; // Absolute time of the beggining of this support state

      double x, y, yaw;  // Position and orientation of the support foot in frame world
      double yawTrunk; /* TODO: Why in SupportState? -
                          -> for compatibility with temporary previewROrientation class*/

      bool stateChanged; /* Defines if the support state has just changed in the FSM. Is true
                            during one QP sample after the state change */

      /// \brief Define if the support state is in a transitional double support phase
      bool inTransitionalDS;

      /// \brief The length of the previous QP sampling period (can be different from QPSamplingPeriod)
      double previousSamplingPeriod;//TODO: change name

      /* \brief The relative weight of each QP sample that falls within this support state.
         (A support state duration of QPSamplingTime have : iterationWeight = 1)*/
      double sampleWeight;//TODO: shouldn't it be outside... somewhere...?
    };

    /* Generic structure, defines a convex hull used to constraint the QP.
       A ConvexHull can be defined in any frame*/
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
      /// \brief Rotation of yaw radians of the convex hull
      void rotate(double yaw);
      // automatically computes the inequalities constraints with the given convex hull
      void computeLinearSystem(const Foot &foot);
    };

    // Parametrization of the MPC scheme
    struct MPC_WALKGEN_API MPCData{
      // The following parameters are fixed once and for all at initialization

      /// \brief Sampling period considered in the QP
      /* Sampling period of the QP, it has to be chosen so that no QP sample falls
         within a transitional double support phase */
      double QPSamplingPeriod;  // blocked - precomputeObjective

      //Feedback period, the time between each call of the MPC
      double MPCSamplingPeriod; /* blocked - precomputeObjective
                                   RigidBodySystem::computeDynamicMatrix */

      /* Period of the samples sent to the actuators. It allows to generate several
         interpolated samples for a single call of the MPC */
      double actuationSamplingPeriod; /* blocked - precomputeObjective
                                         RigidBodySystem::computeDynamicMatrix */

      /// \brief Nb. samplings inside preview window
      int nbSamplesQP;  //blocked - precomputeObjective

      // The following parameters can be changed online
      double stepPeriod; /* Total period of a step
                            blocked by orientPrw_ ? can be solved -- */
      double SSPeriodBeforeDS; /* Used when a reference velocity is smaller than epsilon. It is the
                                 time remaining in SS phase before the robots ends in a DS phase
                                 and stops */
      double firstDStoSSPeriod; /* Time required for the switch of the CoP position
                            during a first DS phase and the SS phase */
      int nbStepsBeforeDS; // Number of steps required before the robot ends in a DS phase and stops


      /// \brief Number of MPC feedbacks before synchronization between MPC sampling and QP sampling
      int nbFeedbackSamplesLeft(double firstSamplingPeriod) const;
      /// \brief number of actuation samples for one MPC feedback
      int nbSamplesControl() const;
      /// \brief number of MPC feedbacks for one QP sample
      int nbFeedbackSamplesStandard() const;

      QPWeighting weighting;

      MPCData();
    };

    //Robot architecture data
    struct MPC_WALKGEN_API RobotData {
      double CoMHeight;
      double freeFlyingFootMaxHeight; //Maximum height the foot can reach during a step

      HipYawData leftHipYaw;
      HipYawData rightHipYaw;

      double robotMass; //Never used //TODO: rename mass

      Eigen::Vector3d leftFootPos; //Initial position of the left foot in frame world
      Eigen::Vector3d rightFootPos; //Initial position of the right foot in frame world

      //Hulls used for constraining the QP
      ConvexHull leftFootHull; // kinematics constraints for the left foot
      ConvexHull rightFootHull; // kinematics constraints for the right foot
      ConvexHull CoPLeftSSHull;
      ConvexHull CoPRightSSHull;
      ConvexHull CoPLeftDSHull;
      ConvexHull CoPRightDSHull;

      RobotData(const HipYawData &leftHipYaw, const HipYawData &rightHipYaw,
                double mass);
      RobotData();
    };

    //Solution of the MPC
    struct MPC_WALKGEN_API MPCSolution{

      MPCSolution();

      void reset();

      // attributes
      /* qpSolution is the vector of the QP solution. Its size is
         2*nbSamplesQP + 2*number of planned steps. It is made of the CoP position along X and Y axis
         for each QP sample of the MPC horizon in the time-dependent local frame. Then we append the planned steps
         positions along X and Y axis in the time-dependent local frame. */
      Eigen::VectorXd qpSolution;
      /* initiaSolution provided by th warmstart to start the QP solver iterative process. */
      Eigen::VectorXd initialSolution;

      Eigen::VectorXi constraints; //Define the active constraints
      Eigen::VectorXi initialConstraints; //Define the active constraints for the initial solution

      bool useWarmStart;

      /// \brief True if a new trajectory is computed in online loop
      bool newTraj;

      /// \brief vector of relative times (starts with 0)
      std::vector<double> samplingTimes_vec;

      std::vector<SupportState> supportStates_vec; /* Vector of the support states planned by the
                                                      MPC scheme (size = number of planned step) */

      //Vectors of support foot and trunk orientation in frame world
      std::vector<double> supportOrientations_vec;
      std::vector<double> supportTrunkOrientations_vec;//TODO: TrunkOrientations_vec

      Eigen::VectorXd CoPTrajX; //CoP trajectory along X axis in frame world
      Eigen::VectorXd CoPTrajY; //CoP trajectory along Y axis in frame world

      struct State
      {
        //These are vectors of size nbSamplesControl()
        Eigen::VectorXd CoMTrajX_; /* CoM trajectory or its derivative or its second
                                      derivative along X axis in frame world */
        Eigen::VectorXd CoMTrajY_; /* CoM trajectory, its derivative or its second
                                      derivative along Y axis in frame world */
        Eigen::VectorXd leftFootTrajX_; /* left foot trajectory or its derivative or its
                                           second derivative along X axis in frame world */
        Eigen::VectorXd leftFootTrajY_; /* left foot trajectory or its derivative or its
                                           second derivative along Y axis in frame world */
        Eigen::VectorXd leftFootTrajZ_; /* left foot trajectory or its derivative or its
                                           second derivative along Z axis in frame world */
        Eigen::VectorXd leftFootTrajYaw_; /* left foot rotation or its derivative or its
                                             second derivative around yaw axis in frame world */
        Eigen::VectorXd rightFootTrajX_; /* right foot trajectory or its derivative or its
                                            second derivative along X axis in frame world */
        Eigen::VectorXd rightFootTrajY_; /* right foot trajectory or its derivative or its
                                            second derivative along Y axis in frame world */
        Eigen::VectorXd rightFootTrajZ_; /* right foot trajectory or its derivative or its
                                            second derivative along Z axis in frame world */
        Eigen::VectorXd rightFootTrajYaw_; /* right foot rotation or its derivative or its
                                              second derivative around yaw axis in frame world */
        Eigen::VectorXd trunkYaw_; /* Trunk rotation, its derivative and second
                                      derivative around yaw axis in frame world */
      };
      std::vector<State> state_vec; /* State vector of size 3, index 0 for position,
                                       index 1 for speed, index 2 for acceleration */

      /*Example: right foot acceleration along Z axis is:
      state_vec[2].rightFootTrajZ_(0), supposing that nbSamplesControl()<1 */
    };
  }
}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif // MPC_WALKGEN_HUMANOID_SHAREDPGTYPE_H
