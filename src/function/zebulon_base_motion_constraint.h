////////////////////////////////////////////////////////////////////////////////
///
///\file	cop_constraint.h
///\brief	Implement the base motion constraints
///\author Lafaye Jory
///\version	1.0
///\date	19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_ZEBULON_BASE_MOTION_CONSTRAINT_H
#define MPC_WALKGEN_ZEBULON_BASE_MOTION_CONSTRAINT_H

#include "../type.h"
#include "../model/zebulon_base_model.h"

namespace MPCWalkgen
{
  class BaseMotionConstraint
  {
  public:
    BaseMotionConstraint(const BaseModel& baseModel);
    ~BaseMotionConstraint();

    const VectorX& getFunctionInf(const VectorX& x0);
    const VectorX& getFunctionSup(const VectorX& x0);
    const MatrixX& getGradient();

    int getNbConstraints();

    void computeConstantPart();

  private:
    void computeFunction(const VectorX& x0, VectorX& func,
                         Scalar velLimit, Scalar accLimit);


  private:
    const BaseModel& baseModel_;

    VectorX functionInf_;
    VectorX functionSup_;
    MatrixX gradient_;
    MatrixX hessian_;

    VectorX tmp_;
    VectorX tmp2_;
  };

}


#endif //MPC_WALKGEN_ZEBULON_BASE_MOTION_CONSTRAINT_H
