#pragma once
#ifndef MPC_WALKGEN_ZEBULON_WAKGEN_H
#define MPC_WALKGEN_ZEBULON_WAKGEN_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	walkgen.h
///\brief	Main program of the MPC
///\author	Lafaye Jory
///\version	1.0
///\date	02/05/12
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/zebulon/walkgen-abstract.h>
#include "types.h"

namespace MPCWalkgen{

  class QPSolver;
  class Interpolation;

  namespace Zebulon{

    class QPGenerator;
    class QPGeneratorOrientation;
    class RigidBodySystem;

    class Walkgen :
        public WalkgenAbstract
    {

    public:
      Walkgen(::MPCWalkgen::QPSolverType solvertype);
      virtual ~Walkgen();

      virtual void robotData(const RobotData &robotData);

      virtual void init(const RobotData &robotData, const MPCData &mpcData);
      virtual void init(const MPCData &mpcData);
      virtual void init(const RobotData &robotData);
      virtual void init();

      virtual const MPCSolution &online(double time, bool previewBodiesNextState=true);

      virtual const MPCSolution &online(bool previewBodiesNextState=true);

      /// \name Accessors and mutators
      /// \{
      virtual void velReferenceInLocalFrame(double dx, double dy, double dyaw);
      virtual void velReferenceInLocalFrame(Eigen::VectorXd dx, Eigen::VectorXd dy, Eigen::VectorXd dyaw);
      virtual void velReferenceInGlobalFrame(double dx, double dy, double dyaw);
      virtual void velReferenceInGlobalFrame(Eigen::VectorXd dx, Eigen::VectorXd dy, Eigen::VectorXd dyaw);
      virtual void posReferenceInGlobalFrame(double dx, double dy, double dyaw);
      virtual void posReferenceInGlobalFrame(Eigen::VectorXd dx, Eigen::VectorXd dy, Eigen::VectorXd dyaw);


      virtual const BodyState & bodyState(BodyType body)const;
      virtual void bodyState(BodyType body, const BodyState & state);
      /// \}


    private:
      MPCData generalData_;
      RobotData robotData_;
      ::MPCWalkgen::QPSolver * solver_;
      ::MPCWalkgen::QPSolver * solverOrientation_;
      QPGenerator * generator_;
      QPGeneratorOrientation * generatorOrientation_;
      ::MPCWalkgen::Interpolation * interpolation_;
      RigidBodySystem * robot_;

      GlobalSolution solution_;
      Reference velRef_;
      Reference posRef_;
      /// \brief The new value of references, updated with in online method
      Reference newVelRef_;
      Reference newPosRef_;


      /// \brief Time at which the problem should be updated
      double upperTimeLimitToUpdate_;
      double upperTimeLimitToFeedback_;

      /// \brief Synchronised time with QP sampling
      double currentTime_;
      double currentRealTime_;

      bool initAlreadyCalled_;

    };
  }
}

#endif // MPC_WALKGEN_ZEBULON_WAKGEN_H
