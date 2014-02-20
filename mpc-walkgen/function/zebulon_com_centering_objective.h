////////////////////////////////////////////////////////////////////////////////
///
///\file com_centering_objective.h
///\brief Implement the com centering objective
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_FUNCTION_ZEBULON_COM_CENTERING_OBJECTIVE_H
#define MPC_WALKGEN_FUNCTION_ZEBULON_COM_CENTERING_OBJECTIVE_H

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
  class ComCenteringObjective
  {
    TEMPLATE_TYPEDEF(Scalar)

  public:
    ComCenteringObjective(const LIPModel<Scalar>& lipModel, const BaseModel<Scalar>& baseModel);
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
    void setNbSamples(int nbSamples);

  private:
    const LIPModel<Scalar>& lipModel_;
    const BaseModel<Scalar>& baseModel_;

    VectorX comRefInLocalFrame_;
    VectorX comShiftInLocalFrame_;
    VectorX gravityShift_;

    VectorX function_;
    MatrixX gradient_;
    MatrixX hessian_;

    VectorX tmp_;
  };

}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
