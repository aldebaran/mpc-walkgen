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

namespace MPCWalkgen
{
  class HumanoidCopConstraint
  {
    public:
      HumanoidCopConstraint();
      ~HumanoidCopConstraint();

      int getNbConstraints() const;

      const VectorX& getFunction(const VectorX& x0);
      const MatrixX& getGradient(const VectorX& x0);

      void computeConstantPart();
  };
}
#endif // MPC_WALKGEN_HUMANOID_COP_CONSTRAINT_H
