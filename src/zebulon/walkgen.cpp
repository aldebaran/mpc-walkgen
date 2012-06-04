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
  ,generalData_()
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

  solver_ = createQPSolver(solvertype,
          4*generalData_.nbSamplesQP, 9*generalData_.nbSamplesQP,
          4*generalData_.nbSamplesQP, 9*generalData_.nbSamplesQP);
  solverOrientation_ = createQPSolver(solvertype,
          generalData_.nbSamplesQP, 2*generalData_.nbSamplesQP,
          generalData_.nbSamplesQP, 2*generalData_.nbSamplesQP);
  interpolation_ = new Interpolation();
  robot_ = new RigidBodySystem(&generalData_, interpolation_);
  generator_= new QPGenerator(solver_, &velRef_, &posRef_, robot_, &generalData_);
  generatorOrientation_= new QPGeneratorOrientation(solverOrientation_, &velRef_, &posRef_, robot_, &generalData_);
}


Walkgen::~Walkgen(){
  if (solverOrientation_ != 0x0)
    delete solverOrientation_;

  if (solver_ != 0x0)
    delete solver_;

  if (generator_ != 0x0)
    delete generator_;

  if (generatorOrientation_ != 0x0)
    delete generatorOrientation_;

  if (robot_ != 0x0)
    delete robot_;

  if (interpolation_ != 0x0)
    delete interpolation_;

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

  if (fabs(robotData.b-tmpRobotData.b)>EPSILON
    ||fabs(robotData.h-tmpRobotData.h)>EPSILON){
    generator_->buildConstraintsCoP();
  }

  if (fabs(robotData.baseLimit[0]-tmpRobotData.baseLimit[0])>EPSILON){
    generator_->buildConstraintsBaseVelocity();
  }
  if (fabs(robotData.baseLimit[1]-tmpRobotData.baseLimit[1])>EPSILON){
    generator_->buildConstraintsBaseAcceleration();
  }
  if (fabs(robotData.baseLimit[2]-tmpRobotData.baseLimit[2])>EPSILON){
    generator_->buildConstraintsBaseJerk();
  }

  if (fabs(robotData.orientationLimit[0]-tmpRobotData.orientationLimit[0])>EPSILON){
    generatorOrientation_->buildConstraintsBaseVelocity();
  }
  if (fabs(robotData.orientationLimit[1]-tmpRobotData.orientationLimit[1])>EPSILON){
    generatorOrientation_->buildConstraintsBaseAcceleration();
  }
  if (fabs(robotData.orientationLimit[2]-tmpRobotData.orientationLimit[2])>EPSILON){
    generatorOrientation_->buildConstraintsBaseJerk();
  }

  if (fabs(robotData.comLimitX-tmpRobotData.comLimitX)>EPSILON
    ||fabs(robotData.comLimitY-tmpRobotData.comLimitY)>EPSILON){
    generator_->buildConstraintsCoM();
  }
}

void Walkgen::init(const RobotData &robotData, const MPCData &mpcData){
  generalData_ = mpcData;
  robotData_ = robotData;
  init();
}

void Walkgen::init(const MPCData &mpcData){
  generalData_ = mpcData;
  init();
}

void Walkgen::init(const RobotData &robotData){
  robotData_ = robotData;
  init();
}

void Walkgen::init() {

  robot_->init(robotData_);

  //Check if sampling periods are defined correctly
  assert(generalData_.actuationSamplingPeriod > 0);
  assert(generalData_.MPCSamplingPeriod >= generalData_.actuationSamplingPeriod);
  assert(generalData_.QPSamplingPeriod >= generalData_.MPCSamplingPeriod);

  robot_->computeDynamics();

  generator_->precomputeObjective();
  generatorOrientation_->precomputeObjective();

  BodyState state;
  robot_->body(BASE)->state(state);

  state.x[0] = 0;
  state.y[0] = 0;
  state.z[0] = robotData_.CoMHeight;
  robot_->body(COM)->state(state);

  currentRealTime_ = 0.0;
  currentTime_ = 0.0;
  upperTimeLimitToUpdate_ = 0.0;
  upperTimeLimitToFeedback_ = 0.0;

  generalData_.ponderation.activePonderation = 0;

  velRef_.resize(generalData_.nbSamplesQP);
  newVelRef_.resize(generalData_.nbSamplesQP);

  posRef_.resize(generalData_.nbSamplesQP);
  newPosRef_.resize(generalData_.nbSamplesQP);

  initAlreadyCalled_ = true;
}

const MPCSolution & Walkgen::online(bool previewBodiesNextState){
  currentRealTime_ += generalData_.MPCSamplingPeriod;
  return online(currentRealTime_, previewBodiesNextState);
}

const MPCSolution & Walkgen::online(double time, bool previewBodiesNextState){
  currentRealTime_ = time;
  solution_.mpcSolution.newTraj = false;
  if(time  > upperTimeLimitToUpdate_+EPSILON){
      upperTimeLimitToUpdate_ += generalData_.QPSamplingPeriod;
      currentTime_ = time;
    }

  if (time  > upperTimeLimitToFeedback_ + EPSILON) {

      solver_->reset();
      solverOrientation_->reset();
      solution_.mpcSolution.newTraj = true;
      velRef_ = newVelRef_;
      posRef_ = newPosRef_;

      upperTimeLimitToFeedback_ += generalData_.MPCSamplingPeriod;

      generatorOrientation_->computeReferenceVector();
      generatorOrientation_->buildObjective();
      generatorOrientation_->buildConstraints();
      generatorOrientation_->computeWarmStart(solution_);

      solverOrientation_->solve(solution_.qpSolutionOrientation,
                                solution_.constraintsOrientation, solution_.initialSolutionOrientation,
                                solution_.initialConstraintsOrientation, solution_.useWarmStart);

      generator_->computeReferenceVector(solution_);
      generator_->buildObjective();
      generator_->buildConstraints();
      generator_->computeWarmStart(solution_);

      solver_->solve(solution_.qpSolution, solution_.constraints,
                     solution_.initialSolution, solution_.initialConstraints, solution_.useWarmStart);

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

const BodyState & Walkgen::bodyState(BodyType body)const{
  return robot_->body(body)->state();
}
void Walkgen::bodyState(BodyType body, const BodyState & state){
  robot_->body(body)->state(state);
}

