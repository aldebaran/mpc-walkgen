////////////////////////////////////////////////////////////////////////////////
///
///\file zebulon_jerk_minimization_objective.h
///\brief Implement the jerk minimization objective
///\author Lafaye Jory
///\date 19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_ZEBULON_JERK_MINIMIZATION_OBJECTIVE_H
#define MPC_WALKGEN_ZEBULON_JERK_MINIMIZATION_OBJECTIVE_H

#include "../type.h"
#include "../model/zebulon_base_model.h"
#include "../model/lip_model.h"

namespace MPCWalkgen
{
  class JerkMinimizationObjective
  {
  public:
    JerkMinimizationObjective(const LIPModel& lipModel, const BaseModel& baseModel);
    ~JerkMinimizationObjective();

    const VectorX& getGradient(const VectorX& x0);
    const MatrixX& getHessian();

    void computeConstantPart();

  private:
    const LIPModel& lipModel_;
    const BaseModel& baseModel_;

    VectorX function_;
    MatrixX gradient_;
    MatrixX hessian_;
  };

}


#endif //MPC_WALKGEN_ZEBULON_JERK_MINIMIZATION_OBJECTIVE_H
