////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_cop_centering_objective.h
///\brief Implement the CoP centering objective
///\author de Gourcuff Martin
///\date 12/07/13
///
////////////////////////////////////////////////////////////////////////////////

#ifndef MPC_WALKGEN_HUMANOID_COP_CENTERING_OBJECTIVE_H
#define MPC_WALKGEN_HUMANOID_COP_CENTERING_OBJECTIVE_H

#include "../type.h"
#include "../model/lip_model.h"
#include "../model/humanoid_foot_model.h"


namespace MPCWalkgen
{
  class HumanoidCopCenteringObjective
  {
    public:
      HumanoidCopCenteringObjective(const LIPModel& lipModel,
                                    const HumanoidFootModel& leftFootModel,
                                    const HumanoidFootModel& rightFootModel);
      ~HumanoidCopCenteringObjective();

      const MatrixX& getGradient(const VectorX& x0);
      const MatrixX& getHessian();
      void computeConstantPart();

    private:
      const LIPModel& lipModel_;
      const HumanoidFootModel& leftFootModel_, rightFootModel_;

      MatrixX gradient_;
      MatrixX hessian_;
  };
}
#endif // MPC_WALKGEN_HUMANOID_COP_CENTERING_OBJECTIVE_H
