#pragma once
#ifndef MPC_WALKGEN_ZEBULON_QP_GENERATOR_H
#define MPC_WALKGEN_ZEBULON_QP_GENERATOR_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	qp-generator.h
///\brief	A class to compute QP elements (objective, constraints, warmstart)
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
    class QPGenerator{

    public:
      QPGenerator(QPSolver * solver, Reference * velRef, Reference * posRef,
                  RigidBodySystem * robot, const MPCData * generalData);
      ~QPGenerator();

      void precomputeObjective();

      void buildObjective();

      void buildConstraints();
      void buildConstraintsCoP();
      void buildConstraintsCoM();
      void buildConstraintsBaseVelocity();
      void buildConstraintsBaseAcceleration();
      void buildConstraintsBaseJerk();

      void computeWarmStart(GlobalSolution & result);

      void computeReferenceVector(const GlobalSolution & result);

    private:

      void computeOrientationMatrices(const GlobalSolution & result);

    private:

      QPSolver * solver_;
      RigidBodySystem * robot_;
      Reference * velRef_;
      Reference * posRef_;
      const MPCData * generalData_;

      Eigen::VectorXd tmpVec_;
      Eigen::VectorXd tmpVec2_;
      Eigen::MatrixXd tmpMat_;

      std::vector<Eigen::MatrixXd> Qconst_;
      std::vector<Eigen::MatrixXd> pconstCoMX_;
      std::vector<Eigen::MatrixXd> pconstCoMB_;
      std::vector<Eigen::MatrixXd> pconstBaseX_;
      std::vector<Eigen::MatrixXd> pconstBaseB_;
      std::vector<Eigen::MatrixXd> pconstVelRef_;
      std::vector<Eigen::MatrixXd> pconstPosRef_;

      Eigen::VectorXd yaw_;
      Eigen::VectorXd cosYaw_;
      Eigen::VectorXd sinYaw_;
      Eigen::MatrixXd Rxx_;
      Eigen::MatrixXd Rxy_;
      Eigen::MatrixXd Ryx_;
      Eigen::MatrixXd Ryy_;

    };
  }
}


#endif // MPC_WALKGEN_ZEBULON_QP_GENERATOR_H
