
#include "qp-generator-orientation.h"
#include "../common/qp-matrix.h"
#include "../common/tools.h"

#include <iostream>
#include <fstream>
#include <cmath>

using namespace MPCWalkgen;
using namespace Zebulon;
using namespace Eigen;

QPGeneratorOrientation::QPGeneratorOrientation(QPSolver * solver,
			 VelReference * velRef, QPPonderation * ponderation,
			 RigidBodySystem * robot, const MPCData * generalData)
  :solver_(solver)
  ,robot_(robot)
  ,velRef_(velRef)
  ,ponderation_(ponderation)
  ,generalData_(generalData)
  ,tmpVec_(1)
{}

QPGeneratorOrientation::~QPGeneratorOrientation(){}


void QPGeneratorOrientation::precomputeObjective(){

  int nbUsedPonderations = ponderation_->baseJerkMin.size();

  Qconst_.resize(nbUsedPonderations);
  pconstCoMYaw_.resize(nbUsedPonderations);
  pconstRef_.resize(nbUsedPonderations);

  int N = generalData_->nbSamplesQP;

  MatrixXd idN = MatrixXd::Identity(N,N);

  const LinearDynamics & CoMVelDynamics = robot_->body(COM)->dynamics(velDynamic);

  for (int i = 0; i < nbUsedPonderations; ++i) {
      Qconst_[i].resize(N,N);
      pconstCoMYaw_[i].resize(N,3);
      pconstRef_[i].resize(N,N);


      Qconst_[i] = ponderation_->OrientationJerkMin[i] * idN
          + ponderation_->OrientationInstantVelocity[i] * CoMVelDynamics.UT*CoMVelDynamics.U;

      pconstRef_[i] = -ponderation_->OrientationInstantVelocity[i] * CoMVelDynamics.UT;
      pconstCoMYaw_[i] = -pconstRef_[i] * CoMVelDynamics.S;

    }
}

void QPGeneratorOrientation::buildObjective() {

  int nb = ponderation_->activePonderation;
  int N = generalData_->nbSamplesQP;

  const BodyState & CoM = robot_->body(COM)->state();

  solver_->nbVar(N);

  solver_->matrix(matrixQ).setTerm(Qconst_[nb]);

  tmpVec_ = pconstCoMYaw_[nb] * CoM.yaw;
  solver_->vector(vectorP).setTerm(tmpVec_);

  tmpVec_ = pconstRef_[nb] * velRef_->global.yaw;
  solver_->vector(vectorP).addTerm(tmpVec_);

}

void QPGeneratorOrientation::buildConstraintsBaseVelocity(){

  int N = generalData_->nbSamplesQP;
  const LinearDynamics & CoMVelDynamics = robot_->body(COM)->dynamics(velDynamic);
  const BodyState & CoM = robot_->body(COM)->state();
  RobotData robotData = robot_->robotData();

  solver_->matrix(matrixA).setTerm(CoMVelDynamics.U, 0, 0);

  tmpVec_.resize(N);
  tmpVec_.fill(-robotData.orientationLimit[0]);
  tmpVec_ -= CoMVelDynamics.S*CoM.yaw;
  solver_->vector(vectorBL).setTerm(tmpVec_, 0);

  tmpVec_.fill(robotData.orientationLimit[0]);
  tmpVec_ -= CoMVelDynamics.S*CoM.yaw;
  solver_->vector(vectorBU).setTerm(tmpVec_, 0);


}

void QPGeneratorOrientation::buildConstraintsBaseAcceleration(){

  int N = generalData_->nbSamplesQP;
  const LinearDynamics & CoMAccDynamics = robot_->body(COM)->dynamics(accDynamic);
  const BodyState & CoM = robot_->body(COM)->state();
  RobotData robotData = robot_->robotData();

  solver_->matrix(matrixA).setTerm(CoMAccDynamics.U, N, 0);

  tmpVec_.resize(N);
  tmpVec_.fill(-robotData.orientationLimit[1]);
  tmpVec_ -= CoMAccDynamics.S*CoM.yaw;
  solver_->vector(vectorBL).setTerm(tmpVec_, N);

  tmpVec_.fill(robotData.orientationLimit[1]);
  tmpVec_ -= CoMAccDynamics.S*CoM.yaw;
  solver_->vector(vectorBU).setTerm(tmpVec_, N);
}

void QPGeneratorOrientation::buildConstraintsBaseJerk(){

  int N = generalData_->nbSamplesQP;
  RobotData robotData = robot_->robotData();

  tmpVec_.resize(N);
  tmpVec_.fill(-robotData.orientationLimit[2]);
  solver_->vector(vectorXL).setTerm(tmpVec_, 0);
  tmpVec_.fill(robotData.orientationLimit[2]);
  solver_->vector(vectorXU).setTerm(tmpVec_, 0);

}

void QPGeneratorOrientation::buildConstraints(){

  solver_->nbCtr(2*generalData_->nbSamplesQP);
  solver_->nbVar(generalData_->nbSamplesQP);

  buildConstraintsBaseVelocity();
  buildConstraintsBaseAcceleration();
  buildConstraintsBaseJerk();

}

void QPGeneratorOrientation::computeWarmStart(MPCSolution & result){
  if (result.constraintsOrientation.rows()>=solver_->nbCtr()+solver_->nbVar()){
      result.initialConstraintsOrientation= result.constraintsOrientation;
      result.initialSolutionOrientation= result.qpSolutionOrientation;
    }else{
      result.initialConstraintsOrientation.setZero((2+1)*generalData_->nbSamplesQP);
      result.initialSolutionOrientation.setZero(generalData_->nbSamplesQP);
    }
}

void QPGeneratorOrientation::computeReferenceVector(){

  if (velRef_->global.yaw.rows()!=generalData_->nbSamplesQP){
    velRef_->global.yaw.resize(generalData_->nbSamplesQP);
  }

  for (int i=0;i<generalData_->nbSamplesQP;++i){
     velRef_->global.yaw(i) = velRef_->local.yaw(i);
  }
}

