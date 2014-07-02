////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_FUNCTION_TRAJECTORY_MOTION_CONSTRAINT_H
#define MPC_WALKGEN_FUNCTION_TRAJECTORY_MOTION_CONSTRAINT_H

#include <mpc-walkgen/type.h>
#include <mpc-walkgen/model/no_dynamic_model.h>

#ifdef _MSC_VER
# pragma warning( push )
// C4251: class needs to have DLL interface
# pragma warning( disable: 4251)
#endif

namespace MPCWalkgen
{
  template <typename Scalar>
  class MotionConstraint
  {
    TEMPLATE_TYPEDEF(Scalar)

  public:
    MotionConstraint(const NoDynamicModel<Scalar>& model);
    ~MotionConstraint();

    const VectorX& getFunctionInf(const VectorX& x0);
    const VectorX& getFunctionSup(const VectorX& x0);
    const MatrixX& getGradient();

    int getNbConstraints();

    void computeConstantPart();

  private:
    void computeFunction(const VectorX& x0, VectorX& func,
                         Scalar velLimit, Scalar accLimit);


  private:
    const NoDynamicModel<Scalar>& model_;

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
