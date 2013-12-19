////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_cop_centering_objective.h
///\brief Implement the CoP centering objective
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_FUNCTION_HUMANOID_COP_CENTERING_OBJECTIVE_H
#define MPC_WALKGEN_FUNCTION_HUMANOID_COP_CENTERING_OBJECTIVE_H

#include <mpc-walkgen/api.h>
#include <mpc-walkgen/type.h>
#include <mpc-walkgen/model/lip_model.h>
#include <mpc-walkgen/humanoid_feet_supervisor.h>

#ifdef _MSC_VER
# pragma warning( push )
// C4251: class needs to have DLL interface
# pragma warning( disable: 4251 )
#endif

namespace MPCWalkgen
{
  template <typename Scalar>
  class MPC_WALKGEN_API HumanoidCopCenteringObjective
  {
    TEMPLATE_TYPEDEF(Scalar)

    public:
      HumanoidCopCenteringObjective(const LIPModel<Scalar>& lipModel,
                                    const HumanoidFeetSupervisor<Scalar>& feetSupervisor);
      ~HumanoidCopCenteringObjective();

      const VectorX& getGradient(const VectorX& x0);
      const MatrixX& getHessian();

    private:
      const LIPModel<Scalar>& lipModel_;
      const HumanoidFeetSupervisor<Scalar>& feetSupervisor_;

      VectorX gradient_;
      MatrixX hessian_;
  };
}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
