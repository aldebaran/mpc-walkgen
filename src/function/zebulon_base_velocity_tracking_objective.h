////////////////////////////////////////////////////////////////////////////////
///
///\file	base_velocity_tracking_objective.h
///\brief	Implement the base velocity tracking objective
///\author Lafaye Jory
///\version	1.0
///\date	19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_ZEBULON_BASE_VELOCITY_TRACKING_OBJECTIVE_H
#define MPC_WALKGEN_ZEBULON_BASE_VELOCITY_TRACKING_OBJECTIVE_H

#include "../type.h"
#include "../model/zebulon_base_model.h"

namespace MPCWalkgen
{
  class BaseVelocityTrackingObjective
  {
  public:
    BaseVelocityTrackingObjective(const BaseModel& baseModel);
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
    const BaseModel& baseModel_;

    VectorX velRefInWorldFrame_;

    VectorX function_;
    MatrixX gradient_;
    MatrixX hessian_;

    VectorX tmp_;
  };

}


#endif //MPC_WALKGEN_ZEBULON_BASE_VELOCITY_TRACKING_OBJECTIVE_H
