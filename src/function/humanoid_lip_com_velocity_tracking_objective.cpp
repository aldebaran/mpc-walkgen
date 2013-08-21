#include "humanoid_lip_com_velocity_tracking_objective.h"

namespace MPCWalkgen
{
  HumanoidLipComVelocityTrackingObjective::HumanoidLipComVelocityTrackingObjective(
      const LIPModel& lipModel,
      const HumanoidFootModel& leftFootModel,
      const HumanoidFootModel& rightFootModel)
    :lipModel_(lipModel)
    ,leftFootModel_(leftFootModel)
    ,rightFootModel_(rightFootModel)
  {
    assert(std::abs(leftFootModel_.getSamplingPeriod() - lipModel_.getSamplingPeriod())<EPSILON);
    assert(std::abs(rightFootModel_.getSamplingPeriod() - lipModel_.getSamplingPeriod())<EPSILON);
    assert(leftFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(rightFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(leftFootModel_.getNbPreviewedSteps() == rightFootModel_.getNbPreviewedSteps());

    velRefInWorldFrame_.setZero(2*lipModel_.getNbSamples());
    gradient_.setZero(1, 1);
    hessian_.setZero(1, 1);
  }

  HumanoidLipComVelocityTrackingObjective::~HumanoidLipComVelocityTrackingObjective(){}

  const MatrixX& HumanoidLipComVelocityTrackingObjective::getGradient(const VectorX& x0)
  {
    assert(velRefInWorldFrame_.size()==2*lipModel_.getNbSamples());
    assert(leftFootModel_.getNbPreviewedSteps()==rightFootModel_.getNbPreviewedSteps());

    int N = lipModel_.getNbSamples();
    int M = leftFootModel_.getNbPreviewedSteps();

    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic();
    const LinearDynamic& dynComVel = lipModel_.getComVelLinearDynamic();

    gradient_.setZero(2*N + 2*M, 1);


    MatrixX tmp = MatrixX::Zero(2*N, 1); //I need a matrix here
    tmp.block(0, 0, N, 1) = (dynComVel.S - dynComVel.U*dynCopX.Uinv*dynCopX.S)
        *lipModel_.getStateX();
    tmp.block(0, 0, N, 1) -= velRefInWorldFrame_.segment(0, N);
    tmp.block(N, 0, N, 1) = (dynComVel.S - dynComVel.U*dynCopY.Uinv*dynCopY.S)
        *lipModel_.getStateY();
    tmp.block(N, 0, N, 1) -= velRefInWorldFrame_.segment(N, N);

    VectorX tmp2 = VectorX::Zero(2*N);
    tmp2.segment(0, N) = dynCopX.UTinv*dynComVel.UT*tmp.block(0, 0, N, 1);
    tmp2.segment(N, N) = dynCopY.UTinv*dynComVel.UT*tmp.block(N, 0, N, 1);

    tmp.setZero(2*N + 2*M, 2*N);
    //Is decomposing the rotation matrix relevant?
    tmp.block(0, 0, 2*N, 2*N) = leftFootModel_.getRotationMatrix()
        + rightFootModel_.getRotationMatrix();
    tmp.block(2*N, 0, M, N) = leftFootModel_.getFootPosLinearDynamic().UT
        + rightFootModel_.getFootPosLinearDynamic().UT;
    tmp.block(2*N + M, N, M, N) = leftFootModel_.getFootPosLinearDynamic().UT
        + rightFootModel_.getFootPosLinearDynamic().UT;

    gradient_ = tmp*tmp2;
    gradient_ += getHessian()*x0;

    return gradient_;
  }

  const MatrixX& HumanoidLipComVelocityTrackingObjective::getHessian()
  {
    assert(leftFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(rightFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(leftFootModel_.getNbPreviewedSteps()==rightFootModel_.getNbPreviewedSteps());

    int N = lipModel_.getNbSamples();
    int M = leftFootModel_.getNbPreviewedSteps();

    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic();
    const LinearDynamic& dynComVel = lipModel_.getComVelLinearDynamic();

    hessian_.setZero(2*N + 2*M, 2*N + 2*M);

    MatrixX tmp = MatrixX::Zero(2*N, 2*N + 2*M);
    tmp.block(0, 0, 2*N, 2*N) = leftFootModel_.getRotationMatrixT()
        + rightFootModel_.getRotationMatrixT(); //Is decomposing the rotation matrix relevant?
    tmp.block(0, 2*N, N, M) = leftFootModel_.getFootPosLinearDynamic().U
        + rightFootModel_.getFootPosLinearDynamic().U;
    tmp.block(N, 2*N + M, N, M) = leftFootModel_.getFootPosLinearDynamic().U
        + rightFootModel_.getFootPosLinearDynamic().U;

    MatrixX tmp2 = MatrixX::Zero(2*N, 2*N);
    tmp2.block(0, 0, N, N) = dynCopX.UTinv*dynComVel.UT*dynComVel.U*dynCopX.Uinv;
    tmp2.block(N, N, N, N) = dynCopY.UTinv*dynComVel.UT*dynComVel.U*dynCopY.Uinv;

    hessian_ = tmp.transpose()*tmp2*tmp;

    return hessian_;
  }

  void HumanoidLipComVelocityTrackingObjective::setVelRefInWorldFrame(const VectorX& velRefInWorldFrame)
  {
    assert(velRefInWorldFrame.size()==lipModel_.getNbSamples()*2);
    assert(velRefInWorldFrame==velRefInWorldFrame);

    velRefInWorldFrame_ = velRefInWorldFrame;
  }
}
