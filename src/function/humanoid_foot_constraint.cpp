#include "humanoid_foot_constraint.h"

namespace MPCWalkgen
{
  HumanoidFootConstraint::HumanoidFootConstraint()
  {
  }

  HumanoidFootConstraint::~HumanoidFootConstraint()
  {
  }

  int HumanoidFootConstraint::getNbConstraints() const
  {
    return 1;
  }

  const VectorX& HumanoidFootConstraint::getFunction(const VectorX& x0)
  {
    VectorX null;
    const VectorX& tmp_return(null);
    return tmp_return;
  }

  const MatrixX& HumanoidFootConstraint::getGradient(const VectorX& x0)
  {
    MatrixX null;
    const MatrixX& tmp_return(null);
    return tmp_return;
  }

  void HumanoidFootConstraint::computeConstantPart()
  {

  }
}
