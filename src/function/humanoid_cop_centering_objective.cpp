////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_cop_centering_objective.cpp
///\brief Implement the CoP centering objective
///\author de Gourcuff Martin
///\date 12/07/13
///
////////////////////////////////////////////////////////////////////////////////

#include "humanoid_cop_centering_objective.h"


namespace MPCWalkgen
{
  HumanoidCopCenteringObjective::HumanoidCopCenteringObjective
  (const LIPModel& lipModel,
   const HumanoidFeetSupervisor& feetSupervisor)
    :lipModel_(lipModel)
    ,feetSupervisor_(feetSupervisor)
  {
    gradient_.setZero(1, 1);
    hessian_.setZero(1, 1);

    computeConstantPart();
  }

  HumanoidCopCenteringObjective::~HumanoidCopCenteringObjective(){}

  const MatrixX& HumanoidCopCenteringObjective::getGradient(const VectorX& x0)
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());
    assert(x0.rows() == 2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    gradient_.noalias() = getHessian()*x0;
    return gradient_;
  }

  const MatrixX& HumanoidCopCenteringObjective::getHessian()
  {
    return hessian_;
  }

  void HumanoidCopCenteringObjective::computeConstantPart()
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());

    unsigned int N = lipModel_.getNbSamples();
    unsigned int M = feetSupervisor_.getNbPreviewedSteps();
    hessian_ = MatrixX::Identity(2*N + 2*M, 2*N + 2*M);

    hessian_.block(2*N, 2*N, 2*M, 2*M) = MatrixX::Zero(2*M, 2*M);
  }
}
