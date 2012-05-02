#include "walkgen.h"

#ifdef USE_QPOASES
# include "../common/qp-solvers/qpoases-solver.h"
#endif //USE_QPOASES

#ifdef USE_LSSOL
# include "../common/qp-solvers/lssol-solver.h"
#endif //USE_LSSOL

#include "qp-generator.h"
#include "rigid-body-system.h"
#include "../common/interpolation.h"

#include <iostream>
#include <Eigen/Dense>

using namespace MPCWalkgen;
using namespace Zebulon;
using namespace Eigen;


MPCWalkgen::Zebulon::WalkgenAbstractZebulon* MPCWalkgen::Zebulon::mpcFactory(MPCWalkgen::QPSolverType solvertype) {
  MPCWalkgen::Zebulon::WalkgenAbstractZebulon* zmpVra = new MPCWalkgen::Zebulon::Walkgen(solvertype);
  return zmpVra;
}


// Implementation of the private interface
Walkgen::Walkgen(::MPCWalkgen::QPSolverType solvertype)
  : WalkgenAbstractZebulon()
  ,generalData_()
  ,solver_(0x0)
  ,solverOrientation_(0x0)
  ,generator_(0x0)
  ,interpolation_(0x0)
  ,robot_(0x0)
  ,solution_()
  ,velRef_()
  ,upperTimeLimitToUpdate_(0)
  ,upperTimeLimitToFeedback_(0)
{

#ifdef USE_QPOASES
  if (solvertype == QPSOLVERTYPE_QPOASES){
      solver_ = new QPOasesSolver(4*generalData_.nbSamplesQP, 9*generalData_.nbSamplesQP,
                                  4*generalData_.nbSamplesQP, 9*generalData_.nbSamplesQP);
      solverOrientation_ = new QPOasesSolver(generalData_.nbSamplesQP, 3*generalData_.nbSamplesQP,
                                             generalData_.nbSamplesQP, 3*generalData_.nbSamplesQP);
    }
#endif //USE_QPOASES

#ifdef USE_LSSOL
  if (solvertype == QPSOLVERTYPE_LSSOL){
      solver_ = new LSSOLSolver(4*generalData_.nbSamplesQP, 9*generalData_.nbSamplesQP,
                                4*generalData_.nbSamplesQP, 9*generalData_.nbSamplesQP);
      solverOrientation_ = new LSSOLSolver(generalData_.nbSamplesQP, 3*generalData_.nbSamplesQP,
                                           generalData_.nbSamplesQP, 3*generalData_.nbSamplesQP);
    }
#endif //USE_LSSOL

  interpolation_ = new Interpolation();

  robot_ = new RigidBodySystem(&generalData_, interpolation_);

  generator_= new QPGenerator(solver_, solverOrientation_, &velRef_, &ponderation_, robot_, &generalData_);

}


Walkgen::~Walkgen(){
  if (solverOrientation_ != 0x0)
    delete solverOrientation_;

  if (solver_ != 0x0)
    delete solver_;

  if (generator_ != 0x0)
    delete generator_;

  if (robot_ != 0x0)
    delete robot_;

  if (interpolation_ != 0x0)
    delete interpolation_;

}

void Walkgen::init(const RobotData &robotData, const MPCData &mpcData) {
  generalData_ = mpcData;
  robot_->init(robotData);

  //Check if sampling periods are defined correctly
  assert(generalData_.actuationSamplingPeriod > 0);
  assert(generalData_.MPCSamplingPeriod >= generalData_.actuationSamplingPeriod);
  assert(generalData_.QPSamplingPeriod >= generalData_.MPCSamplingPeriod);

  robot_->computeDynamics();

  generator_->precomputeObjective();
  generator_->precomputeObjectiveAndConstraintsOrientation();

  BodyState state;

  state.x[0] = robotData.basePos[0];
  state.y[0] = robotData.basePos[1];
  robot_->body(BASE)->state(state);

  state.x[0] = 0;
  state.y[0] = 0;
  state.z[0] = robotData.CoMHeight;
  robot_->body(COM)->state(state);

  upperTimeLimitToUpdate_ = 0.0;
  upperTimeLimitToFeedback_ = 0.0;

  ponderation_.activePonderation = 0;

  velRef_.resize(generalData_.nbSamplesQP);
  newVelRef_.resize(generalData_.nbSamplesQP);
}

const MPCSolution & Walkgen::online(double time, bool previewBodiesNextState){
  solution_.newTraj = false;
  if(time  > upperTimeLimitToUpdate_+EPSILON){
      upperTimeLimitToUpdate_ += generalData_.QPSamplingPeriod;
      currentTime_ = time;
    }

  if (time  > upperTimeLimitToFeedback_ + EPSILON) {
      // UPDATE INTERNAL DATA:
      // ---------------------

      solver_->reset();
      solverOrientation_->reset();
      solution_.reset();
      solution_.newTraj = true;
      velRef_ = newVelRef_;

      upperTimeLimitToFeedback_ += generalData_.MPCSamplingPeriod;

      generator_->computeOrientationReferenceVector();
      generator_->buildObjectiveOrientation();
      generator_->buildConstraintsOrientation();
      generator_->computeWarmStartOrientation(solution_);

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

void Walkgen::QPSamplingPeriod(double)
{
  std::cerr << " The method Walkgen::QPSamplingPeriod(d) is not implemented yet. " << std::endl;
}

void Walkgen::mpcSamplingPeriod(double)
{
  std::cerr << " The method Walkgen::mpcSamplingPeriod(d) is not implemented yet. " << std::endl;
}

void Walkgen::QPNbSamplings(int)
{
  std::cerr << " The method Walkgen::QPNbSamplings(d) is not implemented yet. " << std::endl;
}

const BodyState & Walkgen::bodyState(BodyType body)const{
  return robot_->body(body)->state();
}
void Walkgen::bodyState(BodyType body, const BodyState & state){
  robot_->body(body)->state(state);
}

