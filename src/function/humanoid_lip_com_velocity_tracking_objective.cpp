////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_lip_com_velocity_tracking_objective.cpp
///\brief Implement the LIP CoM velocity tracking objective
///\author de Gourcuff Martin
///\date 12/07/13
///
////////////////////////////////////////////////////////////////////////////////

#include "humanoid_lip_com_velocity_tracking_objective.h"

namespace MPCWalkgen
{
  HumanoidLipComVelocityTrackingObjective::HumanoidLipComVelocityTrackingObjective(
      const LIPModel& lipModel,
      const HumanoidFeetSupervisor &feetSupervisor)
    :lipModel_(lipModel)
    ,feetSupervisor_(feetSupervisor)
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());

    velRefInWorldFrame_.setZero(2*lipModel_.getNbSamples());
    gradient_.setZero(1);
    hessian_.setZero(1, 1);
  }

  HumanoidLipComVelocityTrackingObjective::~HumanoidLipComVelocityTrackingObjective(){}

  const VectorX& HumanoidLipComVelocityTrackingObjective::getGradient(
      const VectorX& x0)
  {
    assert(velRefInWorldFrame_.size()==2*lipModel_.getNbSamples());

    int N = lipModel_.getNbSamples();
    int M = feetSupervisor_.getNbPreviewedSteps();

    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic();
    const LinearDynamic& dynComVel = lipModel_.getComVelLinearDynamic();

    VectorX tmp2 = VectorX::Zero(2*N);

    tmp2.segment(0, N) = dynCopX.UTinv*dynComVel.UT*
        (dynComVel.S*lipModel_.getStateX()
         - dynComVel.U*dynCopX.Uinv*(dynCopX.S*lipModel_.getStateX() + dynCopX.K)
         - velRefInWorldFrame_.segment(0, N));
    tmp2.segment(N, N) = dynCopY.UTinv*dynComVel.UT*
        (dynComVel.S*lipModel_.getStateY()
         - dynComVel.U*dynCopY.Uinv*(dynCopY.S*lipModel_.getStateY() + dynCopY.K)
         - velRefInWorldFrame_.segment(N, N));

    MatrixX tmp = MatrixX::Zero(2*N + 2*M, 2*N);

    tmp.block(0, 0, 2*N, 2*N) = feetSupervisor_.getRotationMatrix();
    tmp.block(2*N, 0, M, N) = feetSupervisor_.getFeetPosLinearDynamic().UT;
    tmp.block(2*N + M, N, M, N) = feetSupervisor_.getFeetPosLinearDynamic().UT;

    gradient_ = tmp*tmp2;
    gradient_ += getHessian()*x0;

    return gradient_;
  }

  const MatrixX& HumanoidLipComVelocityTrackingObjective::getHessian()
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());

    int N = lipModel_.getNbSamples();
    int M = feetSupervisor_.getNbPreviewedSteps();

    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic();
    const LinearDynamic& dynComVel = lipModel_.getComVelLinearDynamic();

    MatrixX tmp = MatrixX::Zero(2*N, 2*N + 2*M);

    tmp.block(0, 0, 2*N, 2*N) = feetSupervisor_.getRotationMatrixT();
    tmp.block(0, 2*N, N, M) = feetSupervisor_.getFeetPosLinearDynamic().U;
    tmp.block(N, 2*N + M, N, M) = feetSupervisor_.getFeetPosLinearDynamic().U;

    MatrixX tmp2 = MatrixX::Zero(2*N, 2*N);

    tmp2.block(0, 0, N, N) = dynCopX.UTinv*dynComVel.UT*dynComVel.U*dynCopX.Uinv;
    tmp2.block(N, N, N, N) = dynCopY.UTinv*dynComVel.UT*dynComVel.U*dynCopY.Uinv;

    hessian_ = tmp.transpose()*tmp2*tmp;

    return hessian_;
  }

  void HumanoidLipComVelocityTrackingObjective::setVelRefInWorldFrame(
      const VectorX& velRefInWorldFrame)
  {
    assert(velRefInWorldFrame.size()==2*lipModel_.getNbSamples());
    assert(velRefInWorldFrame==velRefInWorldFrame);

    velRefInWorldFrame_ = velRefInWorldFrame;
  }
}
