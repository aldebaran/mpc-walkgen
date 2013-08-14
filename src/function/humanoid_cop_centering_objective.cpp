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
    assert(std::abs(leftFootModel_.getSamplingPeriod() - lipModel_.getSamplingPeriod())<EPSILON);
    assert(std::abs(rightFootModel_.getSamplingPeriod() - lipModel_.getSamplingPeriod())<EPSILON);

    gradient_.setZero(1, 1);
    hessian_.setZero(1, 1);

    computeConstantPart();
  }

  HumanoidCopCenteringObjective::~HumanoidCopCenteringObjective(){}

  const MatrixX& HumanoidCopCenteringObjective::getGradient(const VectorX& x0)
  {
    assert(std::abs(leftFootModel_.getSamplingPeriod() - lipModel_.getSamplingPeriod())<EPSILON);
    assert(std::abs(rightFootModel_.getSamplingPeriod() - lipModel_.getSamplingPeriod())<EPSILON);

    gradient_.noalias() = getHessian()*x0;
    return gradient_;
  }

  const MatrixX& HumanoidCopCenteringObjective::getHessian()
  {
    assert(std::abs(leftFootModel_.getSamplingPeriod() - lipModel_.getSamplingPeriod())<EPSILON);
    assert(std::abs(rightFootModel_.getSamplingPeriod() - lipModel_.getSamplingPeriod())<EPSILON);

    return hessian_;
  }

  void HumanoidCopCenteringObjective::computeConstantPart()
  {
    assert(std::abs(leftFootModel_.getSamplingPeriod() - lipModel_.getSamplingPeriod())<EPSILON);
    assert(std::abs(rightFootModel_.getSamplingPeriod() - lipModel_.getSamplingPeriod())<EPSILON);

    int N = lipModel_.getNbSamples();
    int M = leftFootModel_.getNbPreviewedSteps();
    hessian_ = MatrixX::Identity(2*N + 2*M, 2*N + 2*M);

    MatrixX tmp;
    tmp.setZero(2*M, 2*M);

    hessian_.block(2*N, 2*N, 2*M, 2*M) = tmp;
  }
}
