#pragma once
#ifndef MPC_WALKGEN_ZEBULON_QP_GENERATOR_ORIENTATION_H
#define MPC_WALKGEN_ZEBULON_QP_GENERATOR_ORIENTATION_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	qp-generator.h
///\brief	A class to compute QP elements for orientation problem (objective, constraints, warmstart)
///\author	Lafaye Jory
///\version	1.0
///\date	02/05/12
///
////////////////////////////////////////////////////////////////////////////////



#include "types.h"

#include "../common/qp-solver.h"
#include "rigid-body-system.h"

#include <Eigen/Dense>
#include <vector>

namespace MPCWalkgen{
    namespace Zebulon{
    class QPGeneratorOrientation{

    public:
      QPGeneratorOrientation(QPSolver * solver, Reference * velRef,
                  Reference * posRef, RigidBodySystem * robot,
                  const MPCData * generalData);
      ~QPGeneratorOrientation();

      void precomputeObjective();

      void buildObjective();

      void buildConstraints();
      void buildConstraintsBaseVelocity();
      void buildConstraintsBaseAcceleration();
      void buildConstraintsBaseJerk();

      void computeWarmStart(GlobalSolution & result);

      void computeReferenceVector();

    private:

      QPSolver * solver_;
      RigidBodySystem * robot_;
      Reference * velRef_;
      Reference * posRef_;
      const MPCData * generalData_;

      Eigen::VectorXd tmpVec_;

      std::vector<Eigen::MatrixXd> Qconst_;
      std::vector<Eigen::MatrixXd> pconstCoMYaw_;
      std::vector<Eigen::MatrixXd> pconstVelRef_;
      std::vector<Eigen::MatrixXd> pconstPosRef_;

    };
  }
}


#endif // MPC_WALKGEN_ZEBULON_QP_GENERATOR_ORIENTATION_H
