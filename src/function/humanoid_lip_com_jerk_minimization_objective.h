////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_lip_com_jerk_minimization_objective.h
///\brief Implement the LIP CoM jerk minimization objective
///\author de Gourcuff Martin
///\date 12/07/13
///
////////////////////////////////////////////////////////////////////////////////

#ifndef MPC_WALKGEN_HUMANOID_JERK_MINIMIZATION_OBJECTIVE_H
#define MPC_WALKGEN_HUMANOID_JERK_MINIMIZATION_OBJECTIVE_H

#include "../type.h"
#include "../model/lip_model.h"
#include "../model/humanoid_foot_model.h"

namespace MPCWalkgen{
  class HumanoidLipComJerkMinimizationObjective //That's a little bit too long...
  {
    public:
      HumanoidLipComJerkMinimizationObjective(
          const LIPModel& lipModel,
          const HumanoidFootModel& leftFootModel,
          const HumanoidFootModel& rightFootModel);
      ~HumanoidLipComJerkMinimizationObjective();

      const MatrixX& getGradient(const VectorX& x0);
      const MatrixX& getHessian();

      void computeConstantPart();

    private:
      const LIPModel& lipModel_;
      const HumanoidFootModel& leftFootModel_, rightFootModel_;

      MatrixX gradient_;
      MatrixX hessian_;

      //No worries, this will be changed after matrix optim
      MatrixX tmp_;
      MatrixX tmp2_;
      MatrixX tmp3_;
  };
}
#endif // MPC_WALKGEN_HUMANOID_JERK_MINIMIZATION_OBJECTIVE_H
