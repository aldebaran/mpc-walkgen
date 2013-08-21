////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_lip_com_velocity_tracking_objective.h
///\brief Implement the LIP CoM velocity tracking objective
///\author de Gourcuff Martin
///\date 12/07/13
///
////////////////////////////////////////////////////////////////////////////////

#ifndef MPC_WALKGEN_HUMANOID_VELOCITY_TRACKING_OBJECTIVE_H
#define MPC_WALKGEN_HUMANOID_VELOCITY_TRACKING_OBJECTIVE_H

#include "../type.h"
#include "../model/lip_model.h"
#include "../model/humanoid_foot_model.h"

namespace MPCWalkgen{
  class HumanoidLipComVelocityTrackingObjective
  {
    public:
      HumanoidLipComVelocityTrackingObjective(
          const LIPModel& lipModel,
          const HumanoidFootModel& leftFootModel,
          const HumanoidFootModel& rightFootModel);
      ~HumanoidLipComVelocityTrackingObjective();

      const MatrixX& getGradient(const VectorX& x0);
      const MatrixX& getHessian();

      /// \brief Set the base velocity reference in the world frame
      ///        It is a vector of size 2*N, with N the number of samples
      ///        of the LIP model:
      ///        (refX, refY)
      void setVelRefInWorldFrame(const VectorX& velRefInWorldFrame);

    private:
      const LIPModel& lipModel_;
      const HumanoidFootModel& leftFootModel_, rightFootModel_; //Const ref? See Seb

      VectorX velRefInWorldFrame_;

      MatrixX gradient_;
      MatrixX hessian_;
  };
}
#endif // MPC_WALKGEN_HUMANOID_VELOCITY_TRACKING_OBJECTIVE_H
