#include "humanoid_cop_constraint.h"

namespace MPCWalkgen
{
  HumanoidCopConstraint::HumanoidCopConstraint()
  {
  }

  HumanoidCopConstraint::~HumanoidCopConstraint()
  {
  }

  int HumanoidCopConstraint::getNbConstraints() const
  {
    return 1;
  }

  const VectorX& getFunction(const VectorX& x0)
  {
    VectorX null;
    const VectorX& tmp_return(null);
    return tmp_return;
  }

  const MatrixX& HumanoidCopConstraint::getGradient(const VectorX& x0)
  {
    MatrixX null;
    const MatrixX& tmp_return(null);
    return tmp_return;
  }

  void HumanoidCopConstraint::computeConstantPart()
  {

  }
}
