////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_cop_centering_objective.cpp
///\brief Implement the CoP centering objective
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/function/humanoid_cop_centering_objective.h>
#include "../macro.h"

namespace MPCWalkgen
{
  template <typename Scalar>
  HumanoidCopCenteringObjective<Scalar>::HumanoidCopCenteringObjective
  (const LIPModel<Scalar>& lipModel,
   const HumanoidFeetSupervisor<Scalar>& feetSupervisor)
    :lipModel_(lipModel)
    ,feetSupervisor_(feetSupervisor)
  {
    gradient_.setZero(1);
    hessian_.setZero(1, 1);
  }

  template <typename Scalar>
  HumanoidCopCenteringObjective<Scalar>::~HumanoidCopCenteringObjective(){}

  template <typename Scalar>
  const typename
  Type<Scalar>::VectorX& HumanoidCopCenteringObjective<Scalar>::getGradient(const VectorX& x0)
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());
    assert(x0.rows() == 2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    gradient_.noalias() = getHessian()*x0;

    return gradient_;
  }

  template <typename Scalar>
  const typename Type<Scalar>::MatrixX& HumanoidCopCenteringObjective<Scalar>::getHessian()
  {
    int N = lipModel_.getNbSamples();
    int M = feetSupervisor_.getNbPreviewedSteps();

    hessian_ = MatrixX::Identity(2*N + 2*M, 2*N + 2*M);
    hessian_.block(2*N, 2*N, 2*M, 2*M) = MatrixX::Zero(2*M, 2*M);

    return hessian_;
  }

  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(HumanoidCopCenteringObjective);
}
