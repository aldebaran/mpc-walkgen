////////////////////////////////////////////////////////////////////////////////
///
///\file base_motion_constraint.h
///\brief Implement the base motion constraints
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_FUNCTION_ZEBULON_BASE_MOTION_CONSTRAINT_H
#define MPC_WALKGEN_FUNCTION_ZEBULON_BASE_MOTION_CONSTRAINT_H

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
  class BaseMotionConstraint
  {
    TEMPLATE_TYPEDEF(Scalar)

  public:
    BaseMotionConstraint(const BaseModel<Scalar>& baseModel);
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
    const BaseModel<Scalar>& baseModel_;

    VectorX functionInf_;
    VectorX functionSup_;
    MatrixX gradient_;
    MatrixX hessian_;

    VectorX tmp_;
    VectorX tmp2_;
  };
}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
