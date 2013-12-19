////////////////////////////////////////////////////////////////////////////////
///
///\file base_velocity_tracking_objective.h
///\brief Implement the base velocity tracking objective
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_FUNCTION_ZEBULON_BASE_VELOCITY_TRACKING_OBJECTIVE_H
#define MPC_WALKGEN_FUNCTION_ZEBULON_BASE_VELOCITY_TRACKING_OBJECTIVE_H

#include <mpc-walkgen/type.h>
#include <mpc-walkgen/model/zebulon_base_model.h>

#ifdef _MSC_VER
# pragma warning( push )
// C4251: class needs to have DLL interface
# pragma warning( disable: 4251)
#endif

namespace MPCWalkgen
{
  template <typename Scalar>
  class BaseVelocityTrackingObjective
  {
    TEMPLATE_TYPEDEF(Scalar)

  public:
    BaseVelocityTrackingObjective(const BaseModel<Scalar>& baseModel);
    ~BaseVelocityTrackingObjective();

    const MatrixX& getGradient(const VectorX& x0);
    const MatrixX& getHessian();

    /// \brief Set the base velocity reference in the world frame
    ///        It's a vector of size 2*N, where N is the number of samples
    ///        of the base model:
    ///        (refX, refY)
    void setVelRefInWorldFrame(const VectorX& velRefInWorldFrame);

    void computeConstantPart();

  protected:
    const BaseModel<Scalar>& baseModel_;

    VectorX velRefInWorldFrame_;

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
