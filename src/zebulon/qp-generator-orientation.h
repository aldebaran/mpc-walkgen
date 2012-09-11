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
                  Reference * posRef, Reference * posIntRef,
                  RigidBodySystem * robot, const MPCData * generalData,
                  const RobotData * robotData);
      ~QPGeneratorOrientation();

      void precomputeObjective();

      void buildObjective();

      void buildConstraints();
      void buildConstraintsBaseVelocity();
      void buildConstraintsBaseAcceleration();
      void buildConstraintsBaseJerk();

      void computeWarmStart(GlobalSolution & result);

      void computeReferenceVector();

      inline void solver(QPSolver * solver){
        solver_ = solver;
      }

    private:

      QPSolver * solver_;
      RigidBodySystem * robot_;
      Reference * velRef_;
      Reference * posRef_;
      Reference * posIntRef_;
      const MPCData * generalData_;
      const RobotData * robotData_;

      Eigen::VectorXd tmpVec_;

      std::vector<Eigen::MatrixXd> Qconst_;
      std::vector<Eigen::MatrixXd> pconstCoMYaw_;
      std::vector<Eigen::MatrixXd> pconstVelRef_;
      std::vector<Eigen::MatrixXd> pconstPosRef_;

    };
  }
}


#endif // MPC_WALKGEN_ZEBULON_QP_GENERATOR_ORIENTATION_H
