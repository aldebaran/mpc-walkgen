////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_lip_com_velocity_tracking_objective.h
///\brief Implement the LIP CoM velocity tracking objective
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#ifndef MPC_WALKGEN_FUNCTION_HUMANOID_VELOCITY_TRACKING_OBJECTIVE_H
#define MPC_WALKGEN_FUNCTION_HUMANOID_VELOCITY_TRACKING_OBJECTIVE_H

#include <mpc-walkgen/api.h>
#include <mpc-walkgen/type.h>
#include <mpc-walkgen/model/lip_model.h>
#include <mpc-walkgen/humanoid_feet_supervisor.h>

#ifdef _MSC_VER
# pragma warning( push )
// C4251: class needs to have DLL interface
# pragma warning( disable: 4251 )
#endif

namespace MPCWalkgen{
  template <typename Scalar>
  class MPC_WALKGEN_API HumanoidLipComVelocityTrackingObjective
  {
    TEMPLATE_TYPEDEF(Scalar)

    public:
      HumanoidLipComVelocityTrackingObjective(
          const LIPModel<Scalar>& lipModel,
          const HumanoidFeetSupervisor<Scalar>& feetSupervisor);
      ~HumanoidLipComVelocityTrackingObjective();

      const VectorX& getGradient(const VectorX& x0);
      const MatrixX& getHessian();

      /// \brief Set the torso velocity reference in the world frame
      ///        It is a vector of size 2*N, with N the number of samples
      ///        (refX, refY)
      void setVelRefInWorldFrame(const VectorX& velRefInWorldFrame);

    private:
      const LIPModel<Scalar>& lipModel_;
      const HumanoidFeetSupervisor<Scalar>& feetSupervisor_;

      VectorX velRefInWorldFrame_;

      VectorX gradient_;
      MatrixX hessian_;
  };
}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
