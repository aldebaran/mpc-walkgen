////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_FUNCTION_TRAJECTORY_POSITION_TRACKING_OBJECTIVE_H
#define MPC_WALKGEN_FUNCTION_TRAJECTORY_POSITION_TRACKING_OBJECTIVE_H

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
  class PositionTrackingObjective
  {
    TEMPLATE_TYPEDEF(Scalar)

  public:
    PositionTrackingObjective(const NoDynamicModel<Scalar>& model);
    ~PositionTrackingObjective();

    const MatrixX& getGradient(const VectorX& x0);
    const MatrixX& getHessian();

    /// \brief Set the base position reference in the world frame
    void setPosRefInWorldFrame(const VectorX& posRefInWorldFrame);

    void computeConstantPart();

  protected:
    const NoDynamicModel<Scalar>& model_;

    VectorX posRefInWorldFrame_;

    VectorX function_;
    MatrixX gradient_;
    MatrixX hessian_;

    VectorX tmp_;
  };
}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
