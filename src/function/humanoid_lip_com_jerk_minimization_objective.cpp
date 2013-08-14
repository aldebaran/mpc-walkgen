#include "humanoid_lip_com_jerk_minimization_objective.h"

namespace MPCWalkgen
{
  HumanoidLipComJerkMinimizationObjective::HumanoidLipComJerkMinimizationObjective(
      const LIPModel &lipModel,
      const HumanoidFootModel &leftFootModel,
      const HumanoidFootModel &rightFootModel)
    :lipModel_(lipModel)
    ,leftFootModel_(leftFootModel)
    ,rightFootModel_(rightFootModel)
  {
    assert(leftFootModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());
    assert(rightFootModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

    gradient_.setZero(1, 1);
    hessian_.setZero(1, 1);

    computeConstantPart();
  }

  HumanoidLipComJerkMinimizationObjective::~HumanoidLipComJerkMinimizationObjective(){}

  const MatrixX& HumanoidLipComJerkMinimizationObjective::getGradient(const VectorX& x0)
  {

    assert(leftFootModel_.getNbPreviewedSteps()==rightFootModel_.getNbPreviewedSteps());

    int N = lipModel_.getNbSamples();
    int M = leftFootModel_.getNbPreviewedSteps();
    int stateSize = lipModel_.getStateX().rows();

    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic();

    gradient_.setZero(2*N + 2*M, 1);

    //TODO: sparse matrices?
    tmp_.setZero(2*N + 2*M, 2*N);
    tmp_.block(0, 0, 2*N, 2*N) = leftFootModel_.getRotationMatrix()
        + rightFootModel_.getRotationMatrix(); //Is decomposing the rotation matrix relevant?
    tmp_.block(2*N, 0, M, N) = leftFootModel_.getFootPosLinearDynamic().UT
        + rightFootModel_.getFootPosLinearDynamic().UT;
    tmp_.block(2*N + M, N, M, N) = leftFootModel_.getFootPosLinearDynamic().UT
        + rightFootModel_.getFootPosLinearDynamic().UT;

    tmp2_.setZero(2*N, 2*stateSize);
    tmp2_.block(0, 0, N, stateSize)
        = dynCopX.UTinv*dynCopX.Uinv*dynCopX.S;
    tmp2_.block(N, stateSize, N, stateSize)
        = dynCopY.UTinv*dynCopY.Uinv*dynCopY.S;

    tmp3_.setZero(2*stateSize, 1);
    tmp3_.block(0, 0, stateSize, 1) = lipModel_.getStateX();
    tmp3_.block(stateSize, 0, stateSize, 1) = lipModel_.getStateY();

    gradient_ = tmp_*tmp2_*tmp3_;
    gradient_ += getHessian()*x0;
    return gradient_;
  }

  const MatrixX& HumanoidLipComJerkMinimizationObjective::getHessian()
  {
    assert(leftFootModel_.getNbPreviewedSteps()==rightFootModel_.getNbPreviewedSteps());

    int N = lipModel_.getNbSamples();
    int M = leftFootModel_.getNbPreviewedSteps();

    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic();

    hessian_.setZero(2*N + 2*M, 2*N + 2*M);

    tmp_.setZero(2*N, 2*N + 2*M);
    tmp_.block(0, 0, 2*N, 2*N) = leftFootModel_.getRotationMatrixT()
        + rightFootModel_.getRotationMatrixT(); //Is decomposing the rotation matrix relevant?
    tmp_.block(0, 2*N, N, M) = leftFootModel_.getFootPosLinearDynamic().U
        + rightFootModel_.getFootPosLinearDynamic().U;
    tmp_.block(N, 2*N + M, N, M) = leftFootModel_.getFootPosLinearDynamic().U
        + rightFootModel_.getFootPosLinearDynamic().U;

    tmp2_.setZero(2*N, 2*N);
    tmp2_.block(0, 0, N, N) = dynCopX.UTinv*dynCopX.Uinv;
    tmp2_.block(N, N, N, N) = dynCopY.UTinv*dynCopY.Uinv;
    hessian_ = tmp_.transpose()*tmp2_*tmp_;

    return hessian_;
  }

  void HumanoidLipComJerkMinimizationObjective::computeConstantPart()
  {
    //No constant part here
  }
}
