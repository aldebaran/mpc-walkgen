////////////////////////////////////////////////////////////////////////////////
///
///\file cop_constraint.h
///\brief Implement the com constraints
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_FUNCTION_ZEBULON_COM_CONSTRAINT_H
#define MPC_WALKGEN_FUNCTION_ZEBULON_COM_CONSTRAINT_H

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
  class ComConstraint
  {
    TEMPLATE_TYPEDEF(Scalar)

  public:
    ComConstraint(const LIPModel<Scalar>& lipModel, const BaseModel<Scalar>& baseModel);
    ~ComConstraint();

    const VectorX& getFunction(const VectorX& x0);
    const MatrixX& getGradient();

    int getNbConstraints();

    void computeConstantPart();

  private:
    /// \brief Compute the constraints inequalities : A X + b <= 0
    void computeconstraintMatrices();


  private:
    const LIPModel<Scalar>& lipModel_;
    const BaseModel<Scalar>& baseModel_;

    VectorX function_;
    MatrixX gradient_;
    MatrixX hessian_;

    MatrixX A_;
    VectorX b_;

    VectorX tmp_;

  };

}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
