#pragma once
#ifndef MPC_WALKGEN_HUMANOID_FOOT_BODY_H
#define MPC_WALKGEN_HUMANOID_FOOT_BODY_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	com-body.h
///\brief	A class to store CoM rigid body
///\author	Lafaye Jory
///\author      Keith François
///\author	Herdt Andrei
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////

#include "../types.h"
#include "../rigid-body.h"

#include <Eigen/Dense>

namespace MPCWalkgen{
  namespace Humanoid{
    class FootBody:public RigidBody{
    public:
      FootBody(const MPCData * generalData,
               const RobotData * robotData,
               const Interpolation * interpolation, Foot type);
      virtual ~FootBody();

      virtual void interpolate(MPCSolution & result, double currentTime, const Reference & velRef);

    protected:
      virtual void computeDynamicsMatrices(LinearDynamics & dyn,
                                           double S, double T, int N, DynamicMatrixType type);

    private:
      Eigen::VectorXd & getFootVector(MPCSolution & solution, Axis axis, unsigned derivative);

      void computeFootInterpolationByPolynomial(MPCSolution & result, Axis axis, int nbSampling,
                                                const Eigen::Vector3d & FootCurrentState,
                                                double T, const Eigen::Vector3d & nextSupportFootState);

    private:
      Foot footType_;
    };
  }
}

#endif // MPC_WALKGEN_HUMANOID_FOOT_BODY_H
