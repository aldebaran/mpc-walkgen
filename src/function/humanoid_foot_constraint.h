////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_foot_constraint.h
///\brief Implement the foot constraints
///\author de Gourcuff Martin
///\date 12/07/13
///
////////////////////////////////////////////////////////////////////////////////

#ifndef MPC_WALKGEN_HUMANOID_FOOT_CONSTRAINT_H
#define MPC_WALKGEN_HUMANOID_FOOT_CONSTRAINT_H

#include "../type.h"
#include "../model/lip_model.h"
#include "../model/humanoid_foot_model.h"

namespace MPCWalkgen
{
  class HumanoidFootConstraint
  {
    public:
      HumanoidFootConstraint(const LIPModel& lipModel,
                             const HumanoidFootModel& leftFootModel,
                             const HumanoidFootModel& rightFootModel);
      ~HumanoidFootConstraint();

      int getNbConstraints() const;

      const VectorX& getFunction(const VectorX& x0);
      const MatrixX& getGradient();

      void computeConstantPart();
      void computeconstraintMatrices();

    private:
      const LIPModel& lipModel_;
      const HumanoidFootModel& leftFootModel_, rightFootModel_;

      VectorX function_;
      MatrixX gradient_;

      MatrixX A_;
      VectorX b_;
  };
}
#endif // MPC_WALKGEN_HUMANOID_FOOT_CONSTRAINT_H
