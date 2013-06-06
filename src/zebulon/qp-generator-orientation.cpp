
#include "qp-generator-orientation.h"
#include "../common/qp-matrix.h"
#include "../common/tools.h"

#include <iostream>
#include <fstream>
#include <cmath>

using namespace MPCWalkgen;
using namespace Zebulon;
using namespace Eigen;

QPGeneratorOrientation::QPGeneratorOrientation(QPSolver * solver, Reference * velRef,
                                Reference * posRef, Reference * posIntRef,
                                RigidBodySystem * robot, const MPCData * generalData,
                                const RobotData * robotData)
  :solver_(solver)
  ,robot_(robot)
  ,velRef_(velRef)
  ,posRef_(posRef)
  ,posIntRef_(posIntRef)
  ,generalData_(generalData)
  ,robotData_(robotData)
  ,tmpVec_(1)
{}

QPGeneratorOrientation::~QPGeneratorOrientation(){}


void QPGeneratorOrientation::precomputeObjective(){

  int nbUsedWeightings = generalData_->weighting.baseJerkMin.size();

  Qconst_.resize(nbUsedWeightings);
  pconstCoMState.resize(nbUsedWeightings);
  pconstBaseVelRef_.resize(nbUsedWeightings);
  pconstBasePosRef_.resize(nbUsedWeightings);

  int N = generalData_->nbSamplesQP;

  MatrixXd idN = MatrixXd::Identity(N,N);

  const LinearDynamics & CoMPosDynamics = robot_->body(COM)->dynamics(posDynamic);
  const LinearDynamics & CoMVelDynamics = robot_->body(COM)->dynamics(velDynamic);

  for (int i = 0; i < nbUsedWeightings; ++i) {
      Qconst_[i].resize(N,N);
      pconstCoMState[i].resize(N,5);
      pconstBaseVelRef_[i].resize(N,N);
      pconstBasePosRef_[i].resize(N,N);

      Qconst_[i] = generalData_->weighting.OrientationJerkMin[i] * idN
          + generalData_->weighting.OrientationInstantVelocity[i] * CoMVelDynamics.UT*CoMVelDynamics.U
          + generalData_->weighting.OrientationPosition[i] * CoMPosDynamics.UT*CoMPosDynamics.U;

      pconstBaseVelRef_[i] = -generalData_->weighting.OrientationInstantVelocity[i] * CoMVelDynamics.UT;
      pconstBasePosRef_[i] = -generalData_->weighting.OrientationPosition[i] * CoMPosDynamics.UT;
      pconstCoMState[i] = -pconstBaseVelRef_[i] * CoMVelDynamics.S - pconstBasePosRef_[i] * CoMPosDynamics.S;

    }
}

void QPGeneratorOrientation::buildObjective() {

  int nb = generalData_->weighting.activeWeighting;
  int N = generalData_->nbSamplesQP;

  const BodyState & CoM = robot_->body(COM)->state();

  solver_->nbVar(N);
  solver_->matrix(matrixQ).setTerm(Qconst_[nb]);

  tmpVec_ = pconstCoMState[nb] * CoM.yaw;
  solver_->vector(vectorP).setTerm(tmpVec_);

  tmpVec_ = pconstBaseVelRef_[nb] * velRef_->global.yaw;
  solver_->vector(vectorP).addTerm(tmpVec_);

  tmpVec_ = pconstBasePosRef_[nb] * posRef_->global.yaw;
  solver_->vector(vectorP).addTerm(tmpVec_);
}

void QPGeneratorOrientation::buildConstraintsBaseVelocity(){

  int N = generalData_->nbSamplesQP;
  const LinearDynamics & CoMVelDynamics = robot_->body(COM)->dynamics(velDynamic);
  const BodyState & CoM = robot_->body(COM)->state();

  solver_->matrix(matrixA).setTerm(CoMVelDynamics.U, 0, 0);

  tmpVec_.resize(N);
  tmpVec_.fill(-robotData_->orientationLimit[0]);
  tmpVec_ -= CoMVelDynamics.S*CoM.yaw;
  solver_->vector(vectorBL).setTerm(tmpVec_, 0);

  tmpVec_.fill(robotData_->orientationLimit[0]);
  tmpVec_ -= CoMVelDynamics.S*CoM.yaw;
  solver_->vector(vectorBU).setTerm(tmpVec_, 0);


}

void QPGeneratorOrientation::buildConstraintsBaseAcceleration(){

  int N = generalData_->nbSamplesQP;
  const LinearDynamics & CoMAccDynamics = robot_->body(COM)->dynamics(accDynamic);
  const BodyState & CoM = robot_->body(COM)->state();

  solver_->matrix(matrixA).setTerm(CoMAccDynamics.U, N, 0);

  tmpVec_.resize(N);
  tmpVec_.fill(-robotData_->orientationLimit[1]);
  tmpVec_ -= CoMAccDynamics.S*CoM.yaw;
  solver_->vector(vectorBL).setTerm(tmpVec_, N);

  tmpVec_.fill(robotData_->orientationLimit[1]);
  tmpVec_ -= CoMAccDynamics.S*CoM.yaw;
  solver_->vector(vectorBU).setTerm(tmpVec_, N);
}

void QPGeneratorOrientation::buildConstraintsBaseJerk(){

  int N = generalData_->nbSamplesQP;

  tmpVec_.resize(N);
  tmpVec_.fill(-robotData_->orientationLimit[2]);
  solver_->vector(vectorXL).setTerm(tmpVec_, 0);
  tmpVec_.fill(robotData_->orientationLimit[2]);
  solver_->vector(vectorXU).setTerm(tmpVec_, 0);

}

void QPGeneratorOrientation::buildConstraints(){

  solver_->nbCtr(generalData_->QPOrientationNbConstraints);
  solver_->nbVar(generalData_->QPOrientationNbVariables);

  buildConstraintsBaseVelocity();
  buildConstraintsBaseAcceleration();
  buildConstraintsBaseJerk();

}

void QPGeneratorOrientation::computeWarmStart(GlobalSolution & result){
  if (result.constraintsOrientation.rows()>=solver_->nbCtr()+solver_->nbVar()){
      result.initialConstraintsOrientation= result.constraintsOrientation;
      result.initialSolutionOrientation= result.qpSolutionOrientation;
      result.initialLagrangeMultiplierOrientation = result.lagrangeMultiplierOrientation;
    }else{
      result.initialConstraintsOrientation.setZero(generalData_->QPOrientationNbConstraints
                                                 + generalData_->QPOrientationNbVariables);
      result.initialSolutionOrientation.setZero(generalData_->QPOrientationNbVariables);
      result.initialLagrangeMultiplierOrientation.setZero(generalData_->QPOrientationNbConstraints
                                                        + generalData_->QPOrientationNbVariables);
    }
}

void QPGeneratorOrientation::computeReferenceVector(){

  if (velRef_->global.yaw.rows()!=generalData_->nbSamplesQP){
    velRef_->global.yaw.setZero(generalData_->nbSamplesQP);
  }

  for (int i=0;i<generalData_->nbSamplesQP;++i){
     velRef_->global.yaw(i) += velRef_->local.yaw(i);
  }
}

