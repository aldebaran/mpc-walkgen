////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_cop_constraint.h
///\brief Implement the CoP constraint
///\author de Gourcuff Martin
///\date 12/07/13
///
////////////////////////////////////////////////////////////////////////////////

#ifndef MPC_WALKGEN_HUMANOID_COP_CONSTRAINT_H
#define MPC_WALKGEN_HUMANOID_COP_CONSTRAINT_H

#include "../type.h"
#include "../model/lip_model.h"
#include "../model/humanoid_foot_model.h"

namespace MPCWalkgen
{
  class HumanoidCopConstraint
  {
    public:
      HumanoidCopConstraint(const LIPModel& lipModel,
                            const HumanoidFootModel& leftFootModel,
                            const HumanoidFootModel& rightFootModel);
      ~HumanoidCopConstraint();

      int getNbConstraints() const;

      const VectorX& getFunction();

      void computeConstantPart();

    private:
      const LIPModel& lipModel_;
      const HumanoidFootModel& leftFootModel_, rightFootModel_;

      Vector3 ssLeftFootBoundsMin_;
      Vector3 ssLeftFootBoundsMax_;
      Vector3 ssRightFootBoundsMin_;
      Vector3 ssRightFootBoundsMax_;
      Vector3 dsBoundsMin_;
      Vector3 dsBoundsMax_;

      VectorX function_;
  };
}
#endif // MPC_WALKGEN_HUMANOID_COP_CONSTRAINT_H
