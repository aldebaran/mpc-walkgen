////////////////////////////////////////////////////////////////////////////////
///
///\file	cop_centering_objective.h
///\brief	Implement the cop centering objective
///\author Lafaye Jory
///\version	1.0
///\date	19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_ZEBULON_COP_CENTERING_OBJECTIVE_H
#define MPC_WALKGEN_ZEBULON_COP_CENTERING_OBJECTIVE_H

#include "../type.h"
#include "../model/zebulon_base_model.h"
#include "../model/lip_model.h"

namespace MPCWalkgen
{
  class CopCenteringObjective
  {
  public:
    CopCenteringObjective(const LIPModel& lipModel, const BaseModel& baseModel);
    ~CopCenteringObjective();

    const MatrixX& getGradient(const VectorX& x0);
    const MatrixX& getHessian();

    /// \brief Set the CoP reference in the world frame
    ///        It's a vector of size 2*N, where N is the number of samples
    ///        of the lip/base model:
    ///        (refX, refY)
    void setCopRefInLocalFrame(const VectorX& copRefInWorldFrame);

    void computeConstantPart();

  private:
    const LIPModel& lipModel_;
    const BaseModel& baseModel_;

    VectorX copRefInLocalFrame_;

    VectorX function_;
    MatrixX gradient_;
    MatrixX hessian_;

    VectorX tmp_;
  };

}


#endif //MPC_WALKGEN_ZEBULON_COP_CENTERING_OBJECTIVE_H
