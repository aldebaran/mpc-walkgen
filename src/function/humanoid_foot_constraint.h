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

namespace MPCWalkgen
{
  class HumanoidFootConstraint
  {
    public:
      HumanoidFootConstraint();
      ~HumanoidFootConstraint();

      int getNbConstraints() const;

      const VectorX& getFunction(const VectorX& x0);
      const MatrixX& getGradient(const VectorX& x0);

      void computeConstantPart();
  };
}
#endif // MPC_WALKGEN_HUMANOID_FOOT_CONSTRAINT_H
