#include "walkgen.h"

#include "../common/qp-solver.h"
#include "qp-generator.h"
#include "qp-generator-orientation.h"
#include "rigid-body-system.h"
#include "../common/interpolation.h"

#include <iostream>
#include <Eigen/Dense>

using namespace MPCWalkgen;
using namespace Zebulon;
using namespace Eigen;


MPCWalkgen::Zebulon::WalkgenAbstract* MPCWalkgen::Zebulon::createWalkgen(MPCWalkgen::QPSolverType solvertype) {
  MPCWalkgen::Zebulon::WalkgenAbstract* zmpVra = new MPCWalkgen::Zebulon::Walkgen(solvertype);
  return zmpVra;
}


// Implementation of the private interface
Walkgen::Walkgen(::MPCWalkgen::QPSolverType solvertype)
  : WalkgenAbstract()
  ,qpSolverType_(solvertype)
  ,mpcData_()
  ,solver_(0x0)
  ,solverOrientation_(0x0)
  ,generator_(0x0)
  ,generatorOrientation_(0x0)
  ,interpolation_(0x0)
  ,robot_(0x0)
  ,solution_()
  ,velRef_()
  ,upperTimeLimitToUpdate_(0)
  ,upperTimeLimitToFeedback_(0)
  ,initAlreadyCalled_(false)
{
  //Parameters are :
  // *Number min of variables
  // *Number min of constraints
  // *Number max of variables
  // *Number max of constraints
  solver_ = createQPSolver(qpSolverType_,
      mpcData_.QPNbVariables, mpcData_.QPNbConstraints,
      mpcData_.QPNbVariables, mpcData_.QPNbConstraints+EnvData::nbObstacleMax*mpcData_.nbSamplesQP);

  solverOrientation_ = createQPSolver(qpSolverType_,
          mpcData_.QPOrientationNbVariables, mpcData_.QPOrientationNbConstraints,
          mpcData_.QPOrientationNbVariables, mpcData_.QPOrientationNbConstraints);
  interpolation_ = new Interpolation();
  robot_ = new RigidBodySystem(&mpcData_, &robotData_, interpolation_);
  generator_= new QPGenerator(solver_, &velRef_, &posRef_, &posIntRef_,
                              &comRef_, &copRef_, robot_, &mpcData_,
                              &robotData_, &envData_);
  generatorOrientation_= new QPGeneratorOrientation(solverOrientation_, &velRef_, &posRef_,
                                                    &posIntRef_, robot_, &mpcData_, &robotData_);

}


Walkgen::~Walkgen(){
  if (generator_ != 0x0){
    delete generator_;
  }

  if (generatorOrientation_ != 0x0){
    delete generatorOrientation_;
  }

  if (robot_ != 0x0){
    delete robot_;
  }

  if (interpolation_ != 0x0){
    delete interpolation_;
  }

  if (solver_ != 0x0){
    delete solver_;
  }

  if (solverOrientation_ != 0x0){
    delete solverOrientation_;
  }

}

void Walkgen::mpcData(const MPCData &mpcData){
  MPCData tmpMpcData = mpcData_;
  mpcData_ = mpcData;
  mpcData_.QPNbVariables = mpcData_.nbSamplesQP*4;
  mpcData_.QPNbConstraints = mpcData_.nbSamplesQP*9;
  mpcData_.QPOrientationNbVariables = mpcData_.nbSamplesQP;
  mpcData_.QPOrientationNbConstraints = mpcData_.nbSamplesQP*2;

  if (!initAlreadyCalled_){
    init();
    return;
  }

  // Modify dynamic matrices wich are used everywhere in the problem.
  // So, we must to recall init()
  if (mpcData.nbSamplesQP!=tmpMpcData.nbSamplesQP){
    if (solver_ != 0x0){
      delete solver_;
    }
    solver_ = createQPSolver(qpSolverType_,
            mpcData_.QPNbVariables, mpcData_.QPNbConstraints,
            mpcData_.QPNbVariables, mpcData_.QPNbConstraints+EnvData::nbObstacleMax*mpcData_.nbSamplesQP);
    generator_->solver(solver_);

    if (solverOrientation_ != 0x0){
      delete solverOrientation_;
    }
    solverOrientation_ = createQPSolver(qpSolverType_,
            mpcData_.nbSamplesQP, 2*mpcData_.nbSamplesQP,
            mpcData_.nbSamplesQP, 2*mpcData_.nbSamplesQP);
    generatorOrientation_->solver(solverOrientation_);

    init();
    return;
  }

  // Modify dynamic matrices wich are used everywhere in the problem.
  // So, we must to recall init().
  if (fabs(mpcData.QPSamplingPeriod-tmpMpcData.QPSamplingPeriod)>EPSILON){
    init();
    return;
  }

  if (fabs(mpcData.actuationSamplingPeriod-tmpMpcData.actuationSamplingPeriod)>EPSILON
  || fabs(mpcData.MPCSamplingPeriod-tmpMpcData.MPCSamplingPeriod)>EPSILON){
    robot_->computeInterpolationDynamics();
  }

  int size = mpcData.weighting.baseInstantVelocity.size();
  for(int i=0; i<size; ++i){
    if (fabs(mpcData.weighting.baseInstantVelocity[i]-tmpMpcData.weighting.baseInstantVelocity[i])>EPSILON
    || fabs(mpcData.weighting.baseJerkMin[i]-tmpMpcData.weighting.baseJerkMin[i])>EPSILON
    || fabs(mpcData.weighting.basePosition[i]-tmpMpcData.weighting.basePosition[i])>EPSILON
    || fabs(mpcData.weighting.CoMCentering[i]-tmpMpcData.weighting.CoMCentering[i])>EPSILON
    || fabs(mpcData.weighting.CoMJerkMin[i]-tmpMpcData.weighting.CoMJerkMin[i])>EPSILON
    || fabs(mpcData.weighting.angularMomentumMin[i]-tmpMpcData.weighting.angularMomentumMin[i])>EPSILON
    || fabs(mpcData.weighting.CopCentering[i]-tmpMpcData.weighting.CopCentering[i])>EPSILON){
      generator_->precomputeObjective();
      break;
    }
  }

  size = mpcData.weighting.OrientationInstantVelocity.size();
  for(int i=0; i<size; ++i){
    if (fabs(mpcData.weighting.OrientationInstantVelocity[i]-tmpMpcData.weighting.OrientationInstantVelocity[i])>EPSILON
    || fabs(mpcData.weighting.OrientationJerkMin[i]-tmpMpcData.weighting.OrientationJerkMin[i])>EPSILON
    || fabs(mpcData.weighting.OrientationPosition[i]-tmpMpcData.weighting.OrientationPosition[i])>EPSILON){
      generatorOrientation_->precomputeObjective();
      break;
    }
  }
}

void Walkgen::robotData(const RobotData &robotData){
  RobotData tmpRobotData = robotData_;
  robotData_ = robotData;
  if (!initAlreadyCalled_){
    init();
    return;
  }

  // Modify dynamic matrices wich are used everywhere in the problem.
  // So, we must to recall init()
  if (fabs(robotData.CoMHeight-tmpRobotData.CoMHeight)>EPSILON){
    init();
    return;
  }

  if (fabs(robotData.gravity(0)-tmpRobotData.gravity(0))>EPSILON
   || fabs(robotData.gravity(1)-tmpRobotData.gravity(1))>EPSILON
   || fabs(robotData.gravity(2)-tmpRobotData.gravity(2))>EPSILON
   || fabs(robotData.previousGravity(0)-tmpRobotData.previousGravity(0))>EPSILON
   || fabs(robotData.previousGravity(1)-tmpRobotData.previousGravity(1))>EPSILON
   || fabs(robotData.previousGravity(2)-tmpRobotData.previousGravity(2))>EPSILON
  ){
    robot_->computeDynamicsCoP();
    generator_->precomputeObjectiveCoP();
  }

}

void Walkgen::init(const RobotData &robotData, const MPCData &mpcData){
  mpcData_ = mpcData;
  robotData_ = robotData;
  init();
}

void Walkgen::init(const MPCData &mpcData){
  mpcData_ = mpcData;
  init();
}

void Walkgen::init(const RobotData &robotData){
  robotData_ = robotData;
  init();
}

void Walkgen::init() {

  //Check if sampling periods are defined correctly
  assert(mpcData_.actuationSamplingPeriod > 0);
  assert(mpcData_.MPCSamplingPeriod >= mpcData_.actuationSamplingPeriod);
  assert(mpcData_.QPSamplingPeriod >= mpcData_.MPCSamplingPeriod);

  robot_->computeDynamics();
  generator_->precomputeObjective();
  generatorOrientation_->precomputeObjective();

  BodyState state(5);
  state.x[4]=state.y[4]=state.z[4]=state.yaw[4]=1;
  robot_->body(BASE)->state(state);
  state.z[1] = robotData_.CoMHeight;
  robot_->body(COM)->state(state);

  currentRealTime_ = 0.0;
  currentTime_ = 0.0;
  upperTimeLimitToUpdate_ = 0.0;
  upperTimeLimitToFeedback_ = 0.0;

  mpcData_.weighting.activeWeighting = 0;

  velRef_.resize(mpcData_.nbSamplesQP);
  newVelRef_.resize(mpcData_.nbSamplesQP);

  posRef_.resize(mpcData_.nbSamplesQP);
  newPosRef_.resize(mpcData_.nbSamplesQP);

  posIntRef_.resize(mpcData_.nbSamplesQP);
  newPosIntRef_.resize(mpcData_.nbSamplesQP);

  comRef_.resize(mpcData_.nbSamplesQP);
  newcomRef_.resize(mpcData_.nbSamplesQP);

  copRef_.resize(mpcData_.nbSamplesQP);
  newcopRef_.resize(mpcData_.nbSamplesQP);

  initAlreadyCalled_ = true;
}

const MPCSolution & Walkgen::online(bool previewBodiesNextState){
  currentRealTime_ += mpcData_.MPCSamplingPeriod;
  return online(currentRealTime_, previewBodiesNextState);
}

const MPCSolution & Walkgen::online(double time, bool previewBodiesNextState){
  currentRealTime_ = time;
  solution_.mpcSolution.newTraj = false;
  if(time  > upperTimeLimitToUpdate_+EPSILON){
      upperTimeLimitToUpdate_ += mpcData_.QPSamplingPeriod;
      currentTime_ = time;
    }

  if (time  > upperTimeLimitToFeedback_ + EPSILON) {

      solver_->reset();
      solverOrientation_->reset();
      solution_.mpcSolution.newTraj = true;
      velRef_ = newVelRef_;
      posRef_ = newPosRef_;
      posIntRef_ = newPosIntRef_;
      comRef_ = newcomRef_;
      copRef_ = newcopRef_;

      upperTimeLimitToFeedback_ += mpcData_.MPCSamplingPeriod;

      generatorOrientation_->computeReferenceVector();
      generatorOrientation_->buildObjective();
      generatorOrientation_->buildConstraints();
      generatorOrientation_->computeWarmStart(solution_);

      solverOrientation_->solve(solution_.qpSolutionOrientation,
                                solution_.constraintsOrientation,
                                solution_.initialSolutionOrientation,
                                solution_.initialConstraintsOrientation,
                                solution_.initialLagrangeMultiplierOrientation,
                                solution_.lagrangeMultiplierOrientation,
                                solution_.useWarmStart);

      generator_->computeReferenceVector(solution_);
      generator_->computeWarmStart(solution_);
      generator_->buildObjective(solution_);
      generator_->buildConstraints(solution_);


      solution_.mpcSolution.solutionFound = solver_->solve(solution_.qpSolution,
                                                           solution_.constraints,
                                                           solution_.initialSolution,
                                                           solution_.initialConstraints,
                                                           solution_.initialLagrangeMultiplier,
                                                           solution_.lagrangeMultiplier,
                                                           solution_.useWarmStart);


      generator_->computefinalSolution(solution_);

      robot_->interpolateBodies(solution_, time, velRef_);

      if (previewBodiesNextState){
          robot_->updateBodyState(solution_);
        }

    }

  return solution_.mpcSolution;
}

void Walkgen::velReferenceInLocalFrame(double dx, double dy, double dyaw){
  newVelRef_.local.x.fill(dx);
  newVelRef_.local.y.fill(dy);
  newVelRef_.local.yaw.fill(dyaw);
}

void Walkgen::velReferenceInLocalFrame(Eigen::VectorXd dx, Eigen::VectorXd dy, Eigen::VectorXd dyaw){
  newVelRef_.local.x=dx;
  newVelRef_.local.y=dy;
  newVelRef_.local.yaw=dyaw;
}

void Walkgen::velReferenceInGlobalFrame(double dx, double dy, double dyaw){
  newVelRef_.global.x.fill(dx);
  newVelRef_.global.y.fill(dy);
  newVelRef_.global.yaw.fill(dyaw);
}

void Walkgen::velReferenceInGlobalFrame(Eigen::VectorXd dx, Eigen::VectorXd dy, Eigen::VectorXd dyaw){
  newVelRef_.global.x=dx;
  newVelRef_.global.y=dy;
  newVelRef_.global.yaw=dyaw;
}

void Walkgen::posReferenceInGlobalFrame(double dx, double dy, double dyaw){
  newPosRef_.global.x.fill(dx);
  newPosRef_.global.y.fill(dy);
  newPosRef_.global.yaw.fill(dyaw);
}

void Walkgen::posReferenceInGlobalFrame(Eigen::VectorXd dx, Eigen::VectorXd dy, Eigen::VectorXd dyaw){
  newPosRef_.global.x=dx;
  newPosRef_.global.y=dy;
  newPosRef_.global.yaw=dyaw;
}

void Walkgen::posIntReferenceInGlobalFrame(double dx, double dy, double dyaw){
  newPosIntRef_.global.x.fill(dx);
  newPosIntRef_.global.y.fill(dy);
  newPosIntRef_.global.yaw.fill(dyaw);
}

void Walkgen::posIntReferenceInGlobalFrame(Eigen::VectorXd dx, Eigen::VectorXd dy, Eigen::VectorXd dyaw){
  newPosIntRef_.global.x=dx;
  newPosIntRef_.global.y=dy;
  newPosIntRef_.global.yaw=dyaw;
}

void Walkgen::copReferenceInLocalFrame(double dx, double dy){
  newcopRef_.local.x.fill(dx);
  newcopRef_.local.y.fill(dy);
}

void Walkgen::copReferenceInLocalFrame(Eigen::VectorXd dx, Eigen::VectorXd dy){
  newcopRef_.local.x=dx;
  newcopRef_.local.y=dy;
}

void Walkgen::comReferenceInLocalFrame(double dx, double dy){
  newcomRef_.local.x.fill(dx);
  newcomRef_.local.y.fill(dy);
}

void Walkgen::comReferenceInLocalFrame(Eigen::VectorXd dx, Eigen::VectorXd dy){
  newcomRef_.local.x=dx;
  newcomRef_.local.y=dy;
}

const BodyState & Walkgen::bodyState(BodyType body)const{
  return robot_->body(body)->state();
}
void Walkgen::bodyState(BodyType body, const BodyState & state){
  robot_->body(body)->state(state);
}

const MPCData& Walkgen::mpcData(){
  return mpcData_;
}

void Walkgen::envData(const EnvData &envData){
  assert(envData.nbObstacle<=EnvData::nbObstacleMax);
  assert(envData.obstacleLinearizationPointX.size()==mpcData_.nbSamplesQP);
  assert(envData.obstacleLinearizationPointY.size()==mpcData_.nbSamplesQP);
  assert(envData.obstaclePositionX.size()<=EnvData::nbObstacleMax);
  assert(envData.obstaclePositionY.size()==envData.obstaclePositionX.size());
  assert(envData.obstaclePositionY.size()==envData.obstacleRadius.size());

  envData_ = envData;
  mpcData_.QPNbConstraints = mpcData_.nbSamplesQP*(9+envData_.nbObstacle);

}

const RobotData& Walkgen::robotData(){
  return robotData_;
}

const MPCSolution& Walkgen::mpcSolution(){
  return solution_.mpcSolution;
}

const EnvData& Walkgen::envData(){
  return envData_;
}

void Walkgen::QPBasePosition(Eigen::VectorXd& position){
  const int N = mpcData_.nbSamplesQP;
  const LinearDynamics & basePosDynamics = robot_->body(BASE)->dynamics(posDynamic);
  const BodyState & base = robot_->body(BASE)->state();
  position.resize(2*N);

  position.segment(0, N) = basePosDynamics.U * solution_.qpSolution.segment(2*N,N)
                         + basePosDynamics.S * base.x;
  position.segment(N, N) = basePosDynamics.U * solution_.qpSolution.segment(3*N,N)
                         + basePosDynamics.S * base.y;

}
