////////////////////////////////////////////////////////////////////////////////
///
///\file zebulon_tilt_motion_constraint.h
///\brief Implement the tilt motion constraint
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_FUNCTION_ZEBULON_TILT_MOTION_CONSTRAINT_H
#define MPC_WALKGEN_FUNCTION_ZEBULON_TILT_MOTION_CONSTRAINT_H

#include <mpc-walkgen/type.h>
#include <mpc-walkgen/model/zebulon_base_model.h>
#include <mpc-walkgen/model/lip_model.h>

#ifdef _MSC_VER
# pragma warning( push )
// C4251: class needs to have DLL interface
# pragma warning( disable: 4251)
#endif

namespace MPCWalkgen
{
  template <typename Scalar>
  class TiltMotionConstraint
  {
    TEMPLATE_TYPEDEF(Scalar)

  public:
    TiltMotionConstraint(const LIPModel<Scalar>& lipModel, const BaseModel<Scalar>& baseModel);
    ~TiltMotionConstraint();

    const VectorX& getFunction(const VectorX& x0);
    const MatrixX& getGradient();

    int getNbConstraints();

    void computeConstantPart();


  private:
    const LIPModel<Scalar>& lipModel_;
    const BaseModel<Scalar>& baseModel_;

    VectorX function_;
    MatrixX gradient_;
    MatrixX hessian_;
  };

}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
