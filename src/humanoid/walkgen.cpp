#include "walkgen.h"

#include "orientations-preview.h"

#ifdef USE_QPOASES
# include "../common/qp-solvers/qpoases-solver.h"
#endif //USE_QPOASES

#ifdef USE_LSSOL
# include "../common/qp-solvers/lssol-solver.h"
#endif //USE_LSSOL

#include "qp-generator.h"
#include "qp-preview.h"
#include "rigid-body-system.h"
#include "../common/interpolation.h"

#include <iostream>
#include <Eigen/Dense>

using namespace MPCWalkgen;
using namespace Humanoid;
using namespace Eigen;


MPCWalkgen::Humanoid::WalkgenAbstractHumanoid* MPCWalkgen::Humanoid::mpcFactory(MPCWalkgen::QPSolverType solvertype) {
  MPCWalkgen::Humanoid::WalkgenAbstractHumanoid* zmpVra = new MPCWalkgen::Humanoid::Walkgen(solvertype);
  return zmpVra;
}




// Implementation of the private interface
Walkgen::Walkgen(::MPCWalkgen::QPSolverType solvertype)
  : WalkgenAbstractHumanoid()
  ,generalData_()
  ,solver_(0x0)
  ,generator_(0x0)
  ,preview_(0x0)
  ,interpolation_(0x0)
  ,robot_(0x0)
  ,orientPrw_(0x0)
  ,solution_()
  ,velRef_()
  ,newCurrentSupport_()
  ,isNewCurrentSupport_(false)
  ,upperTimeLimitToUpdate_(0)
  ,upperTimeLimitToFeedback_(0)
{

#ifdef USE_QPOASES
  if (solvertype == QPSOLVERTYPE_QPOASES)
    solver_ = new QPOasesSolver(2*generalData_.nbSamplesQP, 0, 2*generalData_.nbSamplesQP+10, 25);
#endif //USE_QPOASES

#ifdef USE_LSSOL
  if (solvertype == QPSOLVERTYPE_LSSOL)
    solver_ = new LSSOLSolver(2*generalData_.nbSamplesQP, 0, 2*generalData_.nbSamplesQP+10, 25);
#endif //USE_LSSOL

  orientPrw_ = new OrientationsPreview();

  interpolation_ = new Interpolation();

  robot_ = new RigidBodySystem(&generalData_, interpolation_);

  preview_ = new QPPreview(&velRef_, robot_, &generalData_);

  generator_= new QPGenerator(preview_, solver_, &velRef_, &ponderation_, robot_, &generalData_);

}


Walkgen::~Walkgen(){
  if (orientPrw_ != 0x0)
    delete orientPrw_;

  if (solver_ != 0x0)
    delete solver_;

  if (generator_ != 0x0)
    delete generator_;

  if (preview_ != 0x0)
    delete preview_;

  if (robot_ != 0x0)
    delete robot_;

  if (interpolation_ != 0x0)
    delete interpolation_;

}

void Walkgen::init(const RobotData &robotData, const MPCData &mpcData) {

  generalData_ = mpcData;
  robotData_ = robotData;

  init();

}

void Walkgen::init() {
  robot_->init(robotData_);

  //Check if sampling periods are defined correctly
  assert(generalData_.actuationSamplingPeriod > 0);
  assert(generalData_.MPCSamplingPeriod >= generalData_.actuationSamplingPeriod);
  assert(generalData_.QPSamplingPeriod >= generalData_.MPCSamplingPeriod);

  // Redistribute the X,Y vectors of variables inside the optimization problems

  VectorXi order(QPSolver::DefaultNbVarMax_);
  for(int i = 0; i < generalData_.nbSamplesQP; ++i) {// 0,2,4,1,3,5 (CoM)
      order(i) = 2*i;
      order(i+generalData_.nbSamplesQP) = 2*i+1;
    }
  for(int i = 2*generalData_.nbSamplesQP; i < QPSolver::DefaultNbVarMax_; ++i) {// 6,7,8 (Feet)
      order(i) = i;
    }
  solver_->varOrder(order);

  orientPrw_->init(generalData_, robotData_);

  robot_->computeDynamics();

  generator_->precomputeObjective();

  BodyState state;
  state.x[0] = robotData_.leftFootPos[0];
  state.y[0] = robotData_.leftFootPos[1];
  robot_->body(LEFT_FOOT)->state(state);

  state.x[0] = robotData_.rightFootPos[0];
  state.y[0] = robotData_.rightFootPos[1];
  robot_->body(RIGHT_FOOT)->state(state);

  state.x[0] = 0;
  state.y[0] = 0;
  state.z[0] = robotData_.CoMHeight;
  robot_->body(COM)->state(state);

  upperTimeLimitToUpdate_ = 0.0;
  upperTimeLimitToFeedback_ = 0.0;

  ponderation_.activePonderation = 0;

  velRef_.resize(generalData_.nbSamplesQP);
  newVelRef_.resize(generalData_.nbSamplesQP);

}

const MPCSolution & Walkgen::online(bool previewBodiesNextState){
  currentRealTime_ += generalData_.MPCSamplingPeriod;
  online(currentRealTime_, previewBodiesNextState);
}

const MPCSolution & Walkgen::online(double time, bool previewBodiesNextState){
  currentRealTime_ = time;
  solution_.newTraj = false;
  if(time  > upperTimeLimitToUpdate_+EPSILON){
      upperTimeLimitToUpdate_ += generalData_.QPSamplingPeriod;
      currentTime_ = time;
    }

  if (time  > upperTimeLimitToFeedback_ + EPSILON) {
      // UPDATE INTERNAL DATA:
      // ---------------------
      solver_->reset();
      solution_.reset();
      solution_.newTraj = true;
      velRef_ = newVelRef_;
      if (isNewCurrentSupport_){
          robot_->currentSupport(newCurrentSupport_);
          isNewCurrentSupport_ = false;
        }

      if (robot_->currentSupport().phase == SS && robot_->currentSupport().nbStepsLeft == 0) {
          velRef_.local.x.fill(0);
          velRef_.local.y.fill(0);
          velRef_.local.yaw.fill(0);
        }
      if (velRef_.local.yaw(0) == 0 && velRef_.local.x(0) == 0 && velRef_.local.y(0) == 0) {
          ponderation_.activePonderation = 1;
        } else {
          ponderation_.activePonderation = 0;
        }

      double firstSamplingPeriod = upperTimeLimitToUpdate_ - upperTimeLimitToFeedback_;
      upperTimeLimitToFeedback_ += generalData_.MPCSamplingPeriod;
      robot_->firstSamplingPeriod(firstSamplingPeriod);

      // PREVIEW:
      // --------
      preview_->previewSamplingTimes(firstSamplingPeriod, solution_);

      preview_->previewSupportStates(currentTime_, firstSamplingPeriod, solution_);

      orientPrw_->preview_orientations( currentTime_, velRef_, generalData_.stepPeriod,
                                        robot_->body(LEFT_FOOT)->state(), robot_->body(RIGHT_FOOT)->state(),
                                        solution_ );
      preview_->computeRotationMatrix(solution_);

      generator_->computeReferenceVector(solution_);
      generator_->buildObjective(solution_);
      generator_->buildConstraints(solution_);
      generator_->computeWarmStart(solution_);

      solver_->solve(solution_.qpSolution,
                     solution_.constraints,
                     solution_.initialSolution,
                     solution_.initialConstraints,
                     solution_.useWarmStart);

      generator_->convertCopToJerk(solution_);

      robot_->interpolateBodies(solution_, time, velRef_);

      if (previewBodiesNextState){
          robot_->updateBodyState(solution_);
        }

      orientPrw_->interpolate_trunk_orientation(robot_);
    }

  return solution_;
}

void Walkgen::reference(double dx, double dy, double dyaw){
  newVelRef_.local.x.fill(dx);
  newVelRef_.local.y.fill(dy);
  newVelRef_.local.yaw.fill(dyaw);
}

void Walkgen::reference(Eigen::VectorXd dx, Eigen::VectorXd dy, Eigen::VectorXd dyaw){
  newVelRef_.local.x=dx;
  newVelRef_.local.y=dy;
  newVelRef_.local.yaw=dyaw;
}


const SupportState & Walkgen::currentSupportState() const {
  return robot_->currentSupport(); }


const BodyState & Walkgen::bodyState(BodyType body)const{
  return robot_->body(body)->state();
}

void Walkgen::bodyState(BodyType body, const BodyState & state){
  robot_->body(body)->state(state);
}

