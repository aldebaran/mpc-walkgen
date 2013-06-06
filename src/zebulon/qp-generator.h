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
                  Reference * posIntRef, Reference * comRef, Reference * copRef, RigidBodySystem * robot,
                  const MPCData * generalData, const RobotData *robotData, const EnvData *envData);
      ~QPGenerator();

      void precomputeObjective();
      void precomputeObjectiveCoP();

      void buildObjective(GlobalSolution &result);

      void buildConstraints(GlobalSolution & result);
      void buildConstraintsCoP();
      void buildConstraintsCoM();
      void buildConstraintsBasePosition(GlobalSolution & result);
      void buildConstraintsBaseVelocity();
      void buildConstraintsBaseAcceleration();
      void buildConstraintsBaseJerk();

      void computeWarmStart(GlobalSolution & result);
      void computefinalSolution(GlobalSolution & result);

      void computeReferenceVector(const GlobalSolution & result);

      inline void solver(QPSolver * solver){
        solver_ = solver;
      }

    private:

      void computeOrientationMatrices(const GlobalSolution & result);

      void computePartOfVectorP(const Eigen::MatrixXd & precomputedMatrix,
                                const Eigen::VectorXd & state,
                                int pos,
                                bool setTerm=false);

      void computePartOfVectorP(const Eigen::MatrixXd & precomputedMatrix,
                                const BodyState & state,
                                int pos,
                                bool setTerm=false);

      void computePartOfVectorP(const Eigen::MatrixXd & precomputedMatrix,
                                const Reference::Frame & state,
                                int pos,
                                bool setTerm=false);
    private:

      QPSolver * solver_;
      RigidBodySystem * robot_;
      Reference * velRef_;
      Reference * posRef_;
      Reference * posIntRef_;
      Reference * comRef_;
      Reference * copRef_;
      const MPCData * generalData_;
      const RobotData * robotData_;
      const EnvData * envData_;

      Eigen::VectorXd tmpVec_;
      Eigen::VectorXd tmpVec2_;
      Eigen::VectorXd tmpVec3_;
      Eigen::VectorXd tmpVec4_;
      Eigen::VectorXd tmpVec5_;
      Eigen::VectorXd tmpVec6_;
      Eigen::MatrixXd tmpMat_;
      Eigen::MatrixXd tmpMat2_;

      // These matrices are the precomputed part of the Hessian matrix Q and the linear vector P of the QP problem.
      // They should be multiplied by a state or reference vector, and then added to the right lines of the matrix/vector
      // (relative to Base or Com variables)
      //
      // Writing convention :
      //
      // [Name of matrix ("Q" "p")]
      // ["CoP" if the matrix is CoP-dynamic-dependant]
      // const
      // [Relative to CoM or Base variables ("ComObj" "BaseObj")]
      // [Must be multiplied by :
      //     All the Com state "ComState"
      //     All the Base state "BaseState"
      //     Com state on axe X/Y "ComStateX" "ComStateY"
      //     Base state on axe X/Y "BaseStateX" "BaseStateY"
      //     Com position reference "ComRef"
      //     CoP position reference "CopRef"
      //     CoP position reference on axe X/Y "CopRefX" "CopRefY"
      //     Base position reference "PosRef"
      //     Base velocity reference "VelREf"
      //     Base position integrale reference "PoseIntRef"]
      //     Base orientation "BaseOrientation"
      std::vector<Eigen::MatrixXd> Qconst_;
      std::vector<Eigen::MatrixXd> QCoPconst_;
      std::vector<Eigen::MatrixXd> pconstComObjComState_;
      std::vector<Eigen::MatrixXd> pCoPconstComObjComStateX_;
      std::vector<Eigen::MatrixXd> pCoPconstComObjComStateY_;
      std::vector<Eigen::MatrixXd> pconstBaseObjComState;
      std::vector<Eigen::MatrixXd> pCoPconstBaseObjComStateX_;
      std::vector<Eigen::MatrixXd> pCoPconstBaseObjComStateY_;
      std::vector<Eigen::MatrixXd> pconstComObjBaseState;
      std::vector<Eigen::MatrixXd> pCoPconstComObjBaseStateX_;
      std::vector<Eigen::MatrixXd> pCoPconstComObjBaseStateY_;
      std::vector<Eigen::MatrixXd> pconstBaseObjBaseState;
      std::vector<Eigen::MatrixXd> pconstBaseObjVelRef_;
      std::vector<Eigen::MatrixXd> pconstBaseObjPosRef_;
      std::vector<Eigen::MatrixXd> pconstBaseObjPosIntRef_;
      std::vector<Eigen::MatrixXd> pCoPconstComObjCopRefX_;
      std::vector<Eigen::MatrixXd> pCoPconstComObjCopRefY_;
      std::vector<Eigen::MatrixXd> pconstBaseObjCopRef_;
      std::vector<Eigen::MatrixXd> pconstComObjComRef_;
      std::vector<Eigen::MatrixXd> pconstBaseObjComRef_;
      std::vector<Eigen::MatrixXd> pconstComObjBaseOrientation_;
      std::vector<Eigen::MatrixXd> pconstBaseObjBaseOrientation_;

      Eigen::VectorXd yaw_;
      Eigen::VectorXd cosYaw_;
      Eigen::VectorXd sinYaw_;
      Eigen::VectorXd Rxx_;
      Eigen::VectorXd Rxy_;
      Eigen::VectorXd Ryx_;
      Eigen::VectorXd Ryy_;

    };
  }
}


#endif // MPC_WALKGEN_ZEBULON_QP_GENERATOR_H
