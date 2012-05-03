#ifndef WALKGENZEBULON
#define WALKGENZEBULON

////////////////////////////////////////////////////////////////////////////////
///
///\file	walkgen.h
///\brief	Main program of the MPC
///\author	Lafaye Jory
///\version	1.0
///\date	02/05/12
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/walkgen-abstract-zebulon.h>
#include "types.h"

namespace MPCWalkgen{

  class QPSolver;
  class Interpolation;

  namespace Zebulon{

    class QPGenerator;
    class RigidBodySystem;

    class Walkgen :
        public WalkgenAbstractZebulon
    {

    public:
      Walkgen(::MPCWalkgen::QPSolverType solvertype);
      ~Walkgen();

      virtual void init(const RobotData &robotData, const MPCData &mpcData);

      virtual const MPCSolution &online(double time, bool previewBodiesNextState=true);

      /// \name Accessors and mutators
      /// \{
      void reference(double dx, double dy, double dyaw);
      void reference(Eigen::VectorXd dx, Eigen::VectorXd dy, Eigen::VectorXd dyaw);


      const BodyState & bodyState(BodyType body)const;
      void bodyState(BodyType body, const BodyState & state);
      /// \}


    private:
      MPCData generalData_;

      ::MPCWalkgen::QPSolver * solver_;
      ::MPCWalkgen::QPSolver * solverOrientation_;
      QPGenerator * generator_;
      ::MPCWalkgen::Interpolation * interpolation_;
      RigidBodySystem * robot_;

      MPCSolution solution_;
      VelReference velRef_;
      /// \brief The new value of reference velocity, updated with in online method
      VelReference newVelRef_;
      QPPonderation ponderation_;


      /// \brief Time at which the problem should be updated
      double upperTimeLimitToUpdate_;
      double upperTimeLimitToFeedback_;

      /// \brief Synchronised time with QP sampling
      double currentTime_;

    };
  }
}

#endif //WALKGENZEBULON
