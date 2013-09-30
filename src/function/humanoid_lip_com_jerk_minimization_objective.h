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
#include "../humanoid_feet_supervisor.h"

namespace MPCWalkgen{
  class HumanoidLipComJerkMinimizationObjective //That's a little bit too long...
  {
    public:
      HumanoidLipComJerkMinimizationObjective(
          const LIPModel& lipModel,
          const HumanoidFeetSupervisor& feetSupervisor);
      ~HumanoidLipComJerkMinimizationObjective();

      const VectorX& getGradient(const VectorX& x0);
      const MatrixX& getHessian();

    private:
      const LIPModel& lipModel_;
      const HumanoidFeetSupervisor& feetSupervisor_;

      VectorX gradient_;
      MatrixX hessian_;
  };
}
#endif // MPC_WALKGEN_HUMANOID_JERK_MINIMIZATION_OBJECTIVE_H
