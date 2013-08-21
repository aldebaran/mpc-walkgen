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
    assert(leftFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(rightFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(leftFootModel_.getNbPreviewedSteps() == rightFootModel_.getNbPreviewedSteps());

    gradient_.setZero(1, 1);
    hessian_.setZero(1, 1);
  }

  HumanoidLipComJerkMinimizationObjective::~HumanoidLipComJerkMinimizationObjective(){}

  const MatrixX& HumanoidLipComJerkMinimizationObjective::getGradient(const VectorX& x0)
  {
    assert(x0.rows() == 2*lipModel_.getNbSamples() + 2*leftFootModel_.getNbPreviewedSteps());

    int N = lipModel_.getNbSamples();
    int M = leftFootModel_.getNbPreviewedSteps();
    int stateSize = lipModel_.getStateX().rows();

    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic();

    gradient_.setZero(2*N + 2*M, 1);

    //TODO: sparse matrices?
    // Ill change these tmp matrices after calculus optimization
    MatrixX tmp = MatrixX::Zero(2*N + 2*M, 2*N);
    tmp.block(0, 0, 2*N, 2*N) = leftFootModel_.getRotationMatrix() +
                                 rightFootModel_.getRotationMatrix();
    //Is decomposing the rotation matrix relevant?
    tmp.block(2*N, 0, M, N) = leftFootModel_.getFootPosLinearDynamic().UT +
                               rightFootModel_.getFootPosLinearDynamic().UT;
    tmp.block(2*N + M, N, M, N) = leftFootModel_.getFootPosLinearDynamic().UT +
                                   rightFootModel_.getFootPosLinearDynamic().UT;

    MatrixX tmp2 = MatrixX::Zero(2*N, 2*stateSize);
    tmp2.block(0, 0, N, stateSize)
        = dynCopX.UTinv*dynCopX.Uinv*dynCopX.S;
    tmp2.block(N, stateSize, N, stateSize)
        = dynCopY.UTinv*dynCopY.Uinv*dynCopY.S;

    VectorX tmp3 = VectorX::Zero(2*stateSize);
    tmp3.segment(0, stateSize) = lipModel_.getStateX();
    tmp3.segment(stateSize, stateSize) = lipModel_.getStateY();

    gradient_ = tmp*tmp2*tmp3;
    gradient_ += getHessian()*x0;
    return gradient_;
  }

  const MatrixX& HumanoidLipComJerkMinimizationObjective::getHessian()
  {
    assert(leftFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(rightFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(leftFootModel_.getNbPreviewedSteps()==rightFootModel_.getNbPreviewedSteps());

    int N = lipModel_.getNbSamples();
    int M = leftFootModel_.getNbPreviewedSteps();

    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic();

    hessian_.setZero(2*N + 2*M, 2*N + 2*M);

    MatrixX tmp = MatrixX::Zero(2*N, 2*N + 2*M);
    //Is decomposing the rotation matrix relevant?
    tmp.block(0, 0, 2*N, 2*N) = leftFootModel_.getRotationMatrixT() +
                                 rightFootModel_.getRotationMatrixT();
    tmp.block(0, 2*N, N, M) = leftFootModel_.getFootPosLinearDynamic().U +
                               rightFootModel_.getFootPosLinearDynamic().U;
    tmp.block(N, 2*N + M, N, M) = leftFootModel_.getFootPosLinearDynamic().U +
                                   rightFootModel_.getFootPosLinearDynamic().U;

    MatrixX tmp2 = MatrixX::Zero(2*N, 2*N);
    tmp2.block(0, 0, N, N) = dynCopX.UTinv*dynCopX.Uinv;
    tmp2.block(N, N, N, N) = dynCopY.UTinv*dynCopY.Uinv;
    hessian_ = tmp.transpose()*tmp2*tmp;

    return hessian_;
  }
}
