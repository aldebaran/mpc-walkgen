////////////////////////////////////////////////////////////////////////////////
///
///\file	base_position_tracking_objective.h
///\brief	Implement the base position tracking objective
///\author Lafaye Jory
///\version	1.0
///\date	19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_ZEBULON_BASE_POSITION_TRACKING_OBJECTIVE_H
#define MPC_WALKGEN_ZEBULON_BASE_POSITION_TRACKING_OBJECTIVE_H

#include "../type.h"
#include "../model/zebulon_base_model.h"

namespace MPCWalkgen
{
  class BasePositionTrackingObjective
  {
  public:
    BasePositionTrackingObjective(const BaseModel& baseModel);
    ~BasePositionTrackingObjective();

    const MatrixX& getGradient(const VectorX& x0);
    const MatrixX& getHessian();

    /// \brief Set the base position reference in the world frame
    ///        It's a vector of size 2*N, where N is the number of samples
    ///        of the base model:
    ///        (refX, refY)
    void setPosRefInWorldFrame(const VectorX& posRefInWorldFrame);

    void computeConstantPart();

  protected:
    const BaseModel& baseModel_;

    VectorX posRefInWorldFrame_;

    VectorX function_;
    MatrixX gradient_;
    MatrixX hessian_;

    VectorX tmp_;
  };

}


#endif //MPC_WALKGEN_ZEBULON_BASE_POSITION_TRACKING_OBJECTIVE_H
