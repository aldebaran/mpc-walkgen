////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_lip_com_jerk_minimization_objective.cpp
///\brief Implement the LIP CoM jerk minimization objective
///\author de Gourcuff Martin
///\date 12/07/13
///
////////////////////////////////////////////////////////////////////////////////

#include "humanoid_lip_com_jerk_minimization_objective.h"

namespace MPCWalkgen
{
  HumanoidLipComJerkMinimizationObjective::HumanoidLipComJerkMinimizationObjective(
      const LIPModel& lipModel,
      const HumanoidFeetSupervisor& feetSupervisor)
    :lipModel_(lipModel)
    ,feetSupervisor_(feetSupervisor)
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());

    gradient_.setZero(1);
    hessian_.setZero(1, 1);
  }

  HumanoidLipComJerkMinimizationObjective::~HumanoidLipComJerkMinimizationObjective(){}

  const VectorX& HumanoidLipComJerkMinimizationObjective::getGradient(const VectorX& x0)
  {
    assert(x0.rows() == 2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    int N = lipModel_.getNbSamples();
    int M = feetSupervisor_.getNbPreviewedSteps();

    int nb = feetSupervisor_.getNbOfCallsBeforeNextSample() - 1;

    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic(nb);
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic(nb);
    const LinearDynamic& dynFeetPos = feetSupervisor_.getFeetPosLinearDynamic();

    const MatrixX& weight = feetSupervisor_.getSampleWeightMatrix();

    //TODO: sparse matrices?
    // Ill change these tmp matrices after calculus optimization
    // I may change some function to avoid copies
    MatrixX tmp = MatrixX::Zero(2*N + 2*M, 2*N);
    tmp.block(0, 0, 2*N, 2*N) = feetSupervisor_.getRotationMatrix();
    tmp.block(2*N, 0, M, N) = feetSupervisor_.getFeetPosLinearDynamic().UT;
    tmp.block(2*N + M, N, M, N) = feetSupervisor_.getFeetPosLinearDynamic().UT;

    VectorX tmp2 = VectorX::Zero(2*N);
    tmp2.segment(0, N) = dynCopX.UTinv*weight*dynCopX.Uinv*dynFeetPos.S
                         *feetSupervisor_.getSupportFootStateX()(0)
                         - dynCopX.UTinv*weight*dynCopX.Uinv
                         *(dynCopX.S*lipModel_.getStateX() + dynCopX.K);

    tmp2.segment(N, N) = dynCopY.UTinv*weight*dynCopY.Uinv*dynFeetPos.S
                         *feetSupervisor_.getSupportFootStateY()(0)
                         - dynCopY.UTinv*weight*dynCopY.Uinv
                         *(dynCopY.S*lipModel_.getStateY() + dynCopY.K);

    gradient_ = tmp*tmp2;
    gradient_ += getHessian()*x0;
    return gradient_;
  }

  const MatrixX& HumanoidLipComJerkMinimizationObjective::getHessian()
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());;

    int N = lipModel_.getNbSamples();
    int M = feetSupervisor_.getNbPreviewedSteps();

    int nb = feetSupervisor_.getNbOfCallsBeforeNextSample() - 1;

    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic(nb);
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic(nb);

    MatrixX tmp = MatrixX::Zero(2*N, 2*N + 2*M);

    tmp.block(0, 0, 2*N, 2*N) = feetSupervisor_.getRotationMatrixT();
    tmp.block(0, 2*N, N, M) = feetSupervisor_.getFeetPosLinearDynamic().U;
    tmp.block(N, 2*N + M, N, M) = feetSupervisor_.getFeetPosLinearDynamic().U;

    MatrixX tmp2 = MatrixX::Zero(2*N, 2*N);

    const MatrixX& weight = feetSupervisor_.getSampleWeightMatrix();

    tmp2.block(0, 0, N, N) = dynCopX.UTinv*weight*dynCopX.Uinv;
    tmp2.block(N, N, N, N) = dynCopY.UTinv*weight*dynCopY.Uinv;
    hessian_.noalias() = tmp.transpose()*tmp2*tmp;

    return hessian_;
  }
}
