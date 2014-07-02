////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_FUNCTION_TRAJECTORY_JERK_MINIMIZATION_OBJECTIVE_H
#define MPC_WALKGEN_FUNCTION_TRAJECTORY_JERK_MINIMIZATION_OBJECTIVE_H

#include <mpc-walkgen/type.h>
#include <mpc-walkgen/model/no_dynamic_model.h>

#ifdef _MSC_VER
# pragma warning( push )
// C4251: class needs to have DLL interface
# pragma warning( disable: 4251)
#endif

namespace MPCWalkgen
{
  template <typename Scalar>
  class TrajectoryJerkMinimizationObjective
  {
    TEMPLATE_TYPEDEF(Scalar)

  public:
    TrajectoryJerkMinimizationObjective(const NoDynamicModel<Scalar>& model);
    ~TrajectoryJerkMinimizationObjective();

    const VectorX& getGradient(const VectorX& x0);
    const MatrixX& getHessian();

    void computeConstantPart();

  private:
    const NoDynamicModel<Scalar>& model_;

    VectorX function_;
    MatrixX gradient_;
    MatrixX hessian_;
  };

}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
