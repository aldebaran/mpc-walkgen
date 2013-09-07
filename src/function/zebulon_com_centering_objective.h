////////////////////////////////////////////////////////////////////////////////
///
///\file com_centering_objective.h
///\brief Implement the com centering objective
///\author Lafaye Jory
///\date 19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_ZEBULON_COM_CENTERING_OBJECTIVE_H
#define MPC_WALKGEN_ZEBULON_COM_CENTERING_OBJECTIVE_H

#include "../type.h"
#include "../model/zebulon_base_model.h"
#include "../model/lip_model.h"

namespace MPCWalkgen
{
  class ComCenteringObjective
  {
  public:
    ComCenteringObjective(const LIPModel& lipModel, const BaseModel& baseModel);
    ~ComCenteringObjective();

    const MatrixX& getGradient(const VectorX& x0);
    const MatrixX& getHessian();

    /// \brief Set the CoP reference in the world frame
    ///        It's a vector of size 2*N, where N is the number of samples
    ///        of the lip/base model:
    ///        (refX, refY)
    void setComRefInLocalFrame(const VectorX& copRefInWorldFrame);

    void computeConstantPart();
    void updateGravityShift();

  private:
    const LIPModel& lipModel_;
    const BaseModel& baseModel_;

    VectorX comRefInLocalFrame_;
    VectorX comShiftInLocalFrame_;
    VectorX gravityShift_;

    VectorX function_;
    MatrixX gradient_;
    MatrixX hessian_;

    VectorX tmp_;
  };

}


#endif //MPC_WALKGEN_ZEBULON_COM_CENTERING_OBJECTIVE_H
