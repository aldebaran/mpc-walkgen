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
    assert(x0.rows() ==
           2*feetSupervisor_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    int N = lipModel_.getNbSamples();
    int M = feetSupervisor_.getNbPreviewedSteps();

    int nb = feetSupervisor_.getNbOfCallsBeforeNextSample() - 1;

    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic(nb);
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic(nb);
    const LinearDynamic& dynComVel = lipModel_.getComVelLinearDynamic(nb);
    const LinearDynamic& dynFeetPos = feetSupervisor_.getFeetPosLinearDynamic();

    const MatrixX& weight = feetSupervisor_.getSampleWeightMatrix();

    MatrixX tmp = MatrixX::Zero(2*N, 1); //I need a matrix here
    VectorX tmp2 = VectorX::Zero(2*N);

    tmp2.segment(0, N) =
        dynComVel.S*lipModel_.getStateX()
        - dynComVel.U*dynCopX.Uinv*(dynCopX.S*lipModel_.getStateX() + dynCopX.K)
        + dynComVel.U*dynCopX.Uinv*dynFeetPos.S*feetSupervisor_.getSupportFootStateX()(0)
        - velRefInWorldFrame_.segment(0, N);
    tmp.block(0, 0, N, 1) = weight*tmp2.block(0, 0, N, 1);

    tmp2.segment(N, N) =
        dynComVel.S*lipModel_.getStateY()
        - dynComVel.U*dynCopY.Uinv*(dynCopY.S*lipModel_.getStateY() + dynCopY.K)
        + dynComVel.U*dynCopY.Uinv*dynFeetPos.S*feetSupervisor_.getSupportFootStateY()(0)
        - velRefInWorldFrame_.segment(N, N);
    tmp.block(N, 0, N, 1) = weight*tmp2.block(N, 0, N, 1);

    tmp2.segment(0, N) = dynCopX.UTinv*dynComVel.UT*tmp.block(0, 0, N, 1);
    tmp2.segment(N, N) = dynCopY.UTinv*dynComVel.UT*tmp.block(N, 0, N, 1);

    tmp.setZero(2*N + 2*M, 2*N);

    tmp.block(0, 0, 2*N, 2*N) = feetSupervisor_.getRotationMatrix();
    tmp.block(2*N, 0, M, N) = dynFeetPos.UT;
    tmp.block(2*N + M, N, M, N) = dynFeetPos.UT;

    gradient_ = tmp*tmp2;
    gradient_ += getHessian()*x0;

    return gradient_;
  }

  const MatrixX& HumanoidLipComVelocityTrackingObjective::getHessian()
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());

    int N = lipModel_.getNbSamples();
    int M = feetSupervisor_.getNbPreviewedSteps();

    int nb = feetSupervisor_.getNbOfCallsBeforeNextSample() - 1;

    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic(nb);
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic(nb);
    const LinearDynamic& dynComVel = lipModel_.getComVelLinearDynamic(nb);
    const LinearDynamic& dynFeetPos = feetSupervisor_.getFeetPosLinearDynamic();

    MatrixX tmp = MatrixX::Zero(2*N, 2*N + 2*M);

    tmp.block(0, 0, 2*N, 2*N) = feetSupervisor_.getRotationMatrixT();
    tmp.block(0, 2*N, N, M) = dynFeetPos.U;
    tmp.block(N, 2*N + M, N, M) = dynFeetPos.U;

    MatrixX tmp2 = MatrixX::Zero(2*N, 2*N);

    const MatrixX& weight = feetSupervisor_.getSampleWeightMatrix();

    tmp2.block(0, 0, N, N) = dynCopX.UTinv*dynComVel.UT*weight*dynComVel.U*dynCopX.Uinv;
    tmp2.block(N, N, N, N) = dynCopY.UTinv*dynComVel.UT*weight*dynComVel.U*dynCopY.Uinv;

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
