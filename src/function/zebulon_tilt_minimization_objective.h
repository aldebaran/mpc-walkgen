////////////////////////////////////////////////////////////////////////////////
///
///\file zebulon_tilt_minimization_objective.h
///\brief Implement the tilt minimization objective
///\author Lafaye Jory
///\version 1.0
///\date 19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_ZEBULON_TILT_MINIMIZATION_OBJECTIVE_H
#define MPC_WALKGEN_ZEBULON_TILT_MINIMIZATION_OBJECTIVE_H

#include "../type.h"
#include "../model/zebulon_base_model.h"
#include "../model/lip_model.h"

namespace MPCWalkgen
{
  class TiltMinimizationObjective
  {
  public:
    TiltMinimizationObjective(const LIPModel& lipModel, const BaseModel& baseModel);
    ~TiltMinimizationObjective();

    const MatrixX& getGradient(const VectorX& x0);
    const MatrixX& getHessian();

    void computeConstantPart();

  private:
    const LIPModel& lipModel_;
    const BaseModel& baseModel_;

    VectorX function_;
    MatrixX gradient_;
    MatrixX hessian_;

    LinearDynamic dynB_;
    LinearDynamic dynC_;

    Scalar M_;
    Scalar m_;
    Scalar L_;
    Scalar theta0_;
    Scalar hc_;
    Scalar sinTheta0_;
    Scalar cosTheta0_;
    Scalar sin2Theta0_;
    Scalar cos2theta0_;

    Scalar normalizationFactor_;

    Scalar partialFactor5_;
    Scalar partialFactor6_;

  };

}


#endif //MPC_WALKGEN_ZEBULON_TILT_MINIMIZATION_OBJECTIVE_H
