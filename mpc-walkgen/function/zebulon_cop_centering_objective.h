////////////////////////////////////////////////////////////////////////////////
///
///\file cop_centering_objective.h
///\brief Implement the cop centering objective
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_FUNCTION_ZEBULON_COP_CENTERING_OBJECTIVE_H
#define MPC_WALKGEN_FUNCTION_ZEBULON_COP_CENTERING_OBJECTIVE_H

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
  class CopCenteringObjective
  {
    TEMPLATE_TYPEDEF(Scalar)

  public:
    CopCenteringObjective(const LIPModel<Scalar>& lipModel, const BaseModel<Scalar>& baseModel);
    ~CopCenteringObjective();

    const MatrixX& getGradient(const VectorX& x0);
    const MatrixX& getHessian();

    /// \brief Set the CoP reference in the world frame
    ///        It's a vector of size 2*N, where N is the number of samples
    ///        of the lip/base model:
    ///        (refX, refY)
    void setCopRefInLocalFrame(const VectorX& copRefInLocalFrame);
    void setCopRefInWorldFrame(const VectorX& comPosRefInWorldFrame,
                               const VectorX& comAccRefInWorldFrame,
                               const VectorX& basePosRefInWorldFrame,
                               const VectorX& baseAccRefInWorldFrame);

    void computeConstantPart();

  private:
    const LIPModel<Scalar>& lipModel_;
    const BaseModel<Scalar>& baseModel_;

    VectorX copRefInLocalFrame_;

    VectorX function_;
    MatrixX gradient_;
    MatrixX hessian_;

    // Temporary computation variables
    VectorX copRef_;
    VectorX constantGravity_;
  };

}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
