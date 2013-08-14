////////////////////////////////////////////////////////////////////////////////
///
///\file cop_constraint.h
///\brief Implement the cop constraints
///\author Lafaye Jory
///\date 19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_ZEBULON_COP_CONSTRAINT_H
#define MPC_WALKGEN_ZEBULON_COP_CONSTRAINT_H

#include "../type.h"
#include "../model/zebulon_base_model.h"
#include "../model/lip_model.h"

namespace MPCWalkgen
{
  class CopConstraint
  {
  public:
    CopConstraint(const LIPModel& lipModel, const BaseModel& baseModel);
    ~CopConstraint();

    const VectorX& getFunction(const VectorX& x0);
    const MatrixX& getGradient();

    int getNbConstraints();

    void computeConstantPart();

  private:
    /// \brief Compute the constraints inequalities : A X + b <= 0
    void computeconstraintMatrices();

  private:
    const LIPModel& lipModel_;
    const BaseModel& baseModel_;

    VectorX function_;
    MatrixX gradient_;
    MatrixX hessian_;

    MatrixX A_;
    VectorX b_;

    VectorX tmp_;

  };

}


#endif //MPC_WALKGEN_ZEBULON_COP_CONSTRAINT_H
