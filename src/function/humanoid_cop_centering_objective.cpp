#include "humanoid_cop_centering_objective.h"


namespace MPCWalkgen
{
  HumanoidCopCenteringObjective::HumanoidCopCenteringObjective
  (const LIPModel& lipModel,
   const HumanoidFootModel& leftFootModel,
   const HumanoidFootModel& rightFootModel)
    :lipModel_(lipModel)
    ,leftFootModel_(leftFootModel)
    ,rightFootModel_(rightFootModel)
  {
    gradient_.setZero(1, 1);
    hessian_.setZero(1, 1);

    computeConstantPart();
  }

  HumanoidCopCenteringObjective::~HumanoidCopCenteringObjective(){}

  const MatrixX& HumanoidCopCenteringObjective::getGradient(const VectorX& x0)
  {
    assert(x0.rows() == 2*lipModel_.getNbSamples() + 2*leftFootModel_.getNbPreviewedSteps());
    gradient_.noalias() = getHessian()*x0;
    return gradient_;
  }

  const MatrixX& HumanoidCopCenteringObjective::getHessian()
  {
    return hessian_;
  }

  void HumanoidCopCenteringObjective::computeConstantPart()
  {
    assert(leftFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(rightFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(leftFootModel_.getNbPreviewedSteps() == rightFootModel_.getNbPreviewedSteps());

    int N = lipModel_.getNbSamples();
    int M = leftFootModel_.getNbPreviewedSteps();
    hessian_ = MatrixX::Identity(2*N + 2*M, 2*N + 2*M);

    hessian_.block(2*N, 2*N, 2*M, 2*M) = MatrixX::Zero(2*M, 2*M);
  }
}
