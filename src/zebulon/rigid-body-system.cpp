#include "rigid-body-system.h"
#include "rigid-bodies/com-body.h"
#include "rigid-bodies/base-body.h"

#include <cmath>

#define PI 3.1415926

using namespace MPCWalkgen;
using namespace Zebulon;
using namespace Eigen;

RigidBodySystem::RigidBodySystem(const MPCData *generalData,
                                 const RobotData * robotData,
                                 const Interpolation *interpolation)
  :generalData_(generalData)
  ,robotData_(robotData)
{
  CoM_ = new CoMBody(generalData_, robotData_, interpolation);
  base_ = new BaseBody(generalData_, robotData_, interpolation);
}

RigidBodySystem::~RigidBodySystem() {
  delete base_;
  delete CoM_;
}

void RigidBodySystem::computeDynamics() {
  computeQPDynamics();
  computeInterpolationDynamics();
}

void RigidBodySystem::computeQPDynamics() {
  CoM_->computeQPDynamics();
  base_->computeQPDynamics();
}

void RigidBodySystem::computeInterpolationDynamics() {
  CoM_->computeInterpolationDynamics();
  base_->computeInterpolationDynamics();
}

void RigidBodySystem::interpolateBodies(GlobalSolution & solution, double currentTime, const Reference & velRef){
  CoM_->interpolate(solution, currentTime, velRef);
  base_->interpolate(solution, currentTime, velRef);
}

void RigidBodySystem::updateBodyState(const GlobalSolution & solution){
  int nextCurrentState = (int)round(generalData_->MPCSamplingPeriod / generalData_->actuationSamplingPeriod)-1;

  BodyState base(5);
  BodyState CoM(5);

  for (int i = 0; i < 4; ++i){
      const MPCSolution::State & currentState = solution.mpcSolution.state_vec[i];
      base.x(i) = currentState.baseTrajX_(nextCurrentState);
      base.y(i) = currentState.baseTrajY_(nextCurrentState);
      base.z(i) = 0;
      base.yaw(i) = currentState.CoMTrajYaw_(nextCurrentState);

      CoM.x(i) = currentState.CoMTrajX_(nextCurrentState);
      CoM.y(i) = currentState.CoMTrajY_(nextCurrentState);
      CoM.yaw(i) = currentState.CoMTrajYaw_(nextCurrentState);
    }
  CoM.z(0) = 0;
  CoM.z(1) = robotData_->CoMHeight;
  CoM.z(2) = 0;
  CoM.z(3) = 0;

  base.yaw(1) = fmod(base.yaw(1),2*PI);
  CoM.yaw(1)  = fmod(CoM.yaw(1),2*PI);

  CoM.x(4)    = 1;
  CoM.y(4)    = 1;
  CoM.z(4)    = 1;
  CoM.yaw(4)  = 1;
  base.x(4)   = 1;
  base.y(4)   = 1;
  base.z(4)   = 1;
  base.yaw(4) = 1;

  CoM_->state(CoM);
  base_->state(base);

}


RigidBody * RigidBodySystem::body(BodyType type){
  switch(type){
    case COM:
      return CoM_;
    default:
      return base_;
    }
}

const RigidBody * RigidBodySystem::body(BodyType type) const{
  switch(type){
    case COM:
      return CoM_;
    default:
      return base_;
    }
}


