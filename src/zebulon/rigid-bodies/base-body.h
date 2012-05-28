#pragma once
#ifndef MPC_WALKGEN_ZEBULON_BASE_BODY_H
#define MPC_WALKGEN_ZEBULON_BASE_BODY_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	base-body.h
///\brief	A class to store Base rigid body
///\author	Lafaye Jory
///\version	1.0
///\date	02/05/12
///
////////////////////////////////////////////////////////////////////////////////

#include "../types.h"
#include "../rigid-body.h"

#include <Eigen/Dense>

namespace MPCWalkgen{
  namespace Zebulon{
    class BaseBody:public RigidBody{
    public:
      BaseBody(const MPCData * generalData,
               const RobotData * robotData,
               const Interpolation * interpolation);
      virtual ~BaseBody();

      virtual void interpolate(GlobalSolution & result, double currentTime, const Reference & velRef);

    protected:
      virtual void computeDynamicsMatrices(LinearDynamics & dyn,
                                           double S, double T, int N, DynamicMatrixType type);

    };
  }
}

#endif //MPC_WALKGEN_ZEBULON_BASE_BODY_H
