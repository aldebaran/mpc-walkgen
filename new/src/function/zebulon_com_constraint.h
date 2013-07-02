////////////////////////////////////////////////////////////////////////////////
///
///\file	cop_constraint.h
///\brief	Implement the com constraints
///\author Lafaye Jory
///\version	1.0
///\date	19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_ZEBULON_COM_CONSTRAINT_H
#define MPC_WALKGEN_ZEBULON_COM_CONSTRAINT_H

#include "../type.h"
#include "../model/zebulon_base_model.h"
#include "../model/lip_model.h"

namespace MPCWalkgen
{
  class ComConstraint
  {
  public:
    ComConstraint(const LIPModel& lipModel, const BaseModel& baseModel);
    ~ComConstraint();

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


#endif //MPC_WALKGEN_ZEBULON_COM_CONSTRAINT_H
