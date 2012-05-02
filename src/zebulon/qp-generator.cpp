#include "qp-generator.h"
#include "../common/qp-matrix.h"
#include "../common/tools.h"

#include <iostream>
#include <fstream>
#include <cmath>

using namespace MPCWalkgen;
using namespace Zebulon;
using namespace Eigen;

QPGenerator::QPGenerator(QPSolver * solver, QPSolver * solverOrientation,
			 VelReference * velRef, QPPonderation * ponderation,
			 RigidBodySystem * robot, const MPCData * generalData)
  :solver_(solver)
  ,solverOrientation_(solverOrientation)
  ,robot_(robot)
  ,velRef_(velRef)
  ,ponderation_(ponderation)
  ,generalData_(generalData)
  ,tmpVec_(1)
  ,tmpVec2_(1)
  ,tmpMat_(1,1)
  ,tmpMat2_(1,1)
{
}

QPGenerator::~QPGenerator(){}


void QPGenerator::precomputeObjective(){

  int nbUsedPonderations = ponderation_->baseJerkMin.size();

  Qconst_.resize(nbUsedPonderations);
  pconstCoMX_.resize(nbUsedPonderations);
  pconstBaseX_.resize(nbUsedPonderations);
  pconstCoMB_.resize(nbUsedPonderations);
  pconstBaseB_.resize(nbUsedPonderations);
  pconstRef_.resize(nbUsedPonderations);

  int N = generalData_->nbSamplesQP;

  MatrixXd idN = MatrixXd::Identity(N,N);

  const LinearDynamics & CoPDynamics = robot_->body(COM)->dynamics(copDynamic);
  const LinearDynamics & CoMPosDynamics = robot_->body(COM)->dynamics(posDynamic);
  const LinearDynamics & basePosDynamics = robot_->body(BASE)->dynamics(posDynamic);
  const LinearDynamics & baseVelDynamics = robot_->body(BASE)->dynamics(velDynamic);


  for (int i = 0; i < nbUsedPonderations; ++i) {
      Qconst_[i].setZero(4*N,4*N);
      pconstCoMX_[i].setZero(N,3);
      pconstBaseX_[i].setZero(N,3);
      pconstCoMB_[i].setZero(N,3);
      pconstBaseB_[i].setZero(N,3);
      pconstRef_[i].setZero(N,N);


      tmpMat_ = ponderation_->CopCentering[i]*CoPDynamics.UT*CoPDynamics.U
          + ponderation_->CoMCentering[i]*CoMPosDynamics.UT*CoMPosDynamics.U
          + ponderation_->CoMJerkMin[i]*idN;
      Qconst_[i].block(0,0,N,N) = tmpMat_;
      Qconst_[i].block(N,N,N,N) = tmpMat_;

      tmpMat_ = -ponderation_->CopCentering[i]*CoPDynamics.UT*basePosDynamics.U
          - ponderation_->CoMCentering[i]*CoMPosDynamics.UT*basePosDynamics.U;
      Qconst_[i].block(0,2*N,N,N) = tmpMat_;
      Qconst_[i].block(N,3*N,N,N) = tmpMat_;
      tmpMat_.transposeInPlace();
      Qconst_[i].block(2*N,0,N,N) = tmpMat_;
      Qconst_[i].block(3*N,N,N,N) = tmpMat_;

      tmpMat_ = (ponderation_->CopCentering[i]+ponderation_->CoMCentering[i])*basePosDynamics.UT*basePosDynamics.U
          + ponderation_->baseInstantVelocity[i]*baseVelDynamics.UT*baseVelDynamics.U
          + ponderation_->baseJerkMin[i]*idN;
      Qconst_[i].block(2*N,2*N,N,N) = tmpMat_;
      Qconst_[i].block(3*N,3*N,N,N) = tmpMat_;



      tmpMat_ = ponderation_->CopCentering[i]*CoPDynamics.UT*CoPDynamics.S
          + ponderation_->CoMCentering[i]*CoMPosDynamics.UT*CoMPosDynamics.S;
      pconstCoMX_[i].block(0,0,N,3) = tmpMat_;

      tmpMat_ = -ponderation_->CopCentering[i]*basePosDynamics.UT*CoPDynamics.S
          - ponderation_->CoMCentering[i]*basePosDynamics.UT*CoMPosDynamics.S;
      pconstCoMB_[i].block(0,0,N,3) = tmpMat_;



      tmpMat_ = -ponderation_->CopCentering[i]*CoPDynamics.UT*basePosDynamics.S
          - ponderation_->CoMCentering[i]*CoMPosDynamics.UT*basePosDynamics.S;
      pconstBaseX_[i].block(0,0,N,3) = tmpMat_;

      tmpMat_ = (ponderation_->CopCentering[i]+ponderation_->CoMCentering[i])*basePosDynamics.UT*basePosDynamics.S
          + ponderation_->baseInstantVelocity[i]*baseVelDynamics.UT*baseVelDynamics.S;
      pconstBaseB_[i].block(0,0,N,3) = tmpMat_;


      tmpMat_ = -ponderation_->baseInstantVelocity[i]*baseVelDynamics.UT;
      pconstRef_[i].block(0,0,N,N) = tmpMat_;

    }
}


void QPGenerator::buildObjective() {

  int nb = ponderation_->activePonderation;
  int N = generalData_->nbSamplesQP;

  const BodyState & CoM = robot_->body(COM)->state();
  const BodyState & base = robot_->body(BASE)->state();

  solver_->nbVar(4*N);
  solver_->nbCtr(0);

  solver_->matrix(matrixQ).addTerm(Qconst_[nb]);

  tmpVec_ = pconstCoMX_[nb] * CoM.x;
  solver_->matrix(vectorP).addTerm(tmpVec_,0,0);
  tmpVec_ = pconstCoMX_[nb] * CoM.y;
  solver_->matrix(vectorP).addTerm(tmpVec_,N,0);

  tmpVec_ = pconstCoMB_[nb] * CoM.x;
  solver_->matrix(vectorP).addTerm(tmpVec_,2*N,0);
  tmpVec_ = pconstCoMB_[nb] * CoM.y;
  solver_->matrix(vectorP).addTerm(tmpVec_,3*N,0);

  tmpVec_ = pconstBaseX_[nb] * base.x;
  solver_->matrix(vectorP).addTerm(tmpVec_,0,0);
  tmpVec_ = pconstBaseX_[nb] * base.y;
  solver_->matrix(vectorP).addTerm(tmpVec_,N,0);

  tmpVec_ = pconstBaseB_[nb] * base.x;
  solver_->matrix(vectorP).addTerm(tmpVec_,2*N,0);
  tmpVec_ = pconstBaseB_[nb] * base.y;
  solver_->matrix(vectorP).addTerm(tmpVec_,3*N,0);
  tmpVec_ = pconstRef_[nb] * velRef_->global.x;
  solver_->matrix(vectorP).addTerm(tmpVec_,2*N,0);
  tmpVec_ = pconstRef_[nb] * velRef_->global.y;
  solver_->matrix(vectorP).addTerm(tmpVec_,3*N,0);

}


void QPGenerator::buildConstraints(){

  int N = generalData_->nbSamplesQP;
  int nbCtr = solver_->nbCtr();
  solver_->addNbCtr(9*N);

  MatrixXd idN = MatrixXd::Identity(N,N);


  const LinearDynamics & CoPDynamics = robot_->body(COM)->dynamics(copDynamic);
  const LinearDynamics & basePosDynamics = robot_->body(BASE)->dynamics(posDynamic);
  const LinearDynamics & baseVelDynamics = robot_->body(BASE)->dynamics(velDynamic);
  const LinearDynamics & baseAccDynamics = robot_->body(BASE)->dynamics(accDynamic);

  const BodyState & CoM = robot_->body(COM)->state();
  const BodyState & base = robot_->body(BASE)->state();

  RobotData robotData = robot_->robotData();

  tmpMat_.resize(N,N);

  tmpMat_ = -Rxx_*CoPDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+0, 0);

  tmpMat_ = -Rxx_*CoPDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+N, 0);

  tmpMat_ = Rxx_*CoPDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+2*N, 0);

  tmpMat_ = -Rxy_*CoPDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+0, N);

  tmpMat_ = -Rxy_*CoPDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+N, N);

  tmpMat_ = Rxy_*CoPDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+2*N, N);


  tmpMat_ = Ryx_*(2*robotData.h/robotData.b)*CoPDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+0, 0);

  tmpMat_ = -Ryx_*(2*robotData.h/robotData.b)*CoPDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+N, 0);

  tmpMat_ = Ryy_*(2*robotData.h/robotData.b)*CoPDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+0, N);

  tmpMat_ = -Ryy_*(2*robotData.h/robotData.b)*CoPDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+N, N);



  tmpMat_ = Rxx_*basePosDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+0, 2*N);

  tmpMat_ = Rxx_*basePosDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+N, 2*N);

  tmpMat_ = -Rxx_*basePosDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+2*N, 2*N);

  tmpMat_ = Rxy_*basePosDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+0, 3*N);

  tmpMat_ = Rxy_*basePosDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+N, 3*N);

  tmpMat_ = -Rxy_*basePosDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+2*N, 3*N);



  tmpMat_ = -Ryx_*(2*robotData.h/robotData.b)*basePosDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+0, 2*N);

  tmpMat_ = Ryx_*(2*robotData.h/robotData.b)*basePosDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+N, 2*N);

  tmpMat_ = -Ryy_*(2*robotData.h/robotData.b)*basePosDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+0, 3*N);

  tmpMat_ = Ryy_*(2*robotData.h/robotData.b)*basePosDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+N, 3*N);



  tmpMat_ = baseVelDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+3*N, 2*N);

  tmpMat_ = baseVelDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+4*N, 3*N);

  tmpMat_ = baseAccDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+5*N, 2*N);

  tmpMat_ = baseAccDynamics.U;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+6*N, 3*N);

  tmpMat_ = idN;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+7*N, 2*N);

  tmpMat_ = idN;
  solver_->matrix(matrixA).addTerm(tmpMat_, nbCtr+8*N, 3*N);




  tmpVec_.resize(9*N);

  tmpVec_.segment(0,3*N).fill(-10e11);
  tmpVec_.segment(3*N,2*N).fill(-robotData.baseLimit[0]);
  tmpVec_.segment(5*N,2*N).fill(-robotData.baseLimit[1]);
  tmpVec_.segment(7*N,2*N).fill(-robotData.baseLimit[2]);

  tmpVec_.segment(3*N,N) -= baseVelDynamics.S*base.x;
  tmpVec_.segment(4*N,N) -= baseVelDynamics.S*base.y;

  tmpVec_.segment(5*N,N) -= baseAccDynamics.S*base.x;
  tmpVec_.segment(6*N,N) -= baseAccDynamics.S*base.y;

  solver_->matrix(vectorBL).addTerm(tmpVec_, nbCtr);



  tmpVec_.segment(0,3*N).fill(robotData.h/2);
  tmpVec_.segment(0,N) += Rxx_*CoPDynamics.S * CoM.x - Ryy_*(2*robotData.h/robotData.b)*CoPDynamics.S* CoM.y - Rxx_*basePosDynamics.S*base.x + Ryy_*(2*robotData.h/robotData.b)* basePosDynamics.S*base.y;
  tmpVec_.segment(N,N) += Rxx_*CoPDynamics.S * CoM.x + Ryy_*(2*robotData.h/robotData.b)*CoPDynamics.S* CoM.y - Rxx_*basePosDynamics.S*base.x - Ryy_*(2*robotData.h/robotData.b)* basePosDynamics.S*base.y;
  tmpVec_.segment(2*N,N) += -Rxx_*CoPDynamics.S*CoM.x + Rxx_*basePosDynamics.S*base.x;

  tmpVec_.segment(0,N) += Rxy_*CoPDynamics.S * CoM.y - Ryx_*(2*robotData.h/robotData.b)*CoPDynamics.S* CoM.x - Rxy_*basePosDynamics.S*base.y + Ryx_*(2*robotData.h/robotData.b)* basePosDynamics.S*base.x;
  tmpVec_.segment(    N,N) += Rxy_*CoPDynamics.S * CoM.y + Ryx_*(2*robotData.h/robotData.b)*CoPDynamics.S* CoM.x - Rxy_*basePosDynamics.S*base.y - Ryx_*(2*robotData.h/robotData.b)* basePosDynamics.S*base.x;
  tmpVec_.segment(2*N,N) += -Rxy_*CoPDynamics.S*CoM.y + Rxy_*basePosDynamics.S*base.y;


  tmpVec_.segment(3*N,2*N).fill(robotData.baseLimit[0]);
  tmpVec_.segment(5*N,2*N).fill(robotData.baseLimit[1]);
  tmpVec_.segment(7*N,2*N).fill(robotData.baseLimit[2]);

  tmpVec_.segment(3*N,N) -= baseVelDynamics.S*base.x;
  tmpVec_.segment(4*N,N) -= baseVelDynamics.S*base.y;

  tmpVec_.segment(5*N,N) -= baseAccDynamics.S*base.x;
  tmpVec_.segment(6*N,N) -= baseAccDynamics.S*base.y;

  solver_->matrix(vectorBU).addTerm(tmpVec_, nbCtr);



  tmpVec_.resize(4*N);
  tmpVec_.fill(-10e11);
  solver_->matrix(vectorXL).addTerm(tmpVec_, nbCtr);
  tmpVec_.fill(10e11);
  solver_->matrix(vectorXU).addTerm(tmpVec_, nbCtr);
}

void QPGenerator::computeWarmStart(MPCSolution & result){
  if (result.constraints.rows()>=solver_->nbCtr()+solver_->nbVar()){
      result.initialConstraints= result.constraints;
      result.initialSolution= result.qpSolution;
    }else{
      result.initialConstraints.setZero((9+4)*generalData_->nbSamplesQP);
      result.initialSolution.setZero(4*generalData_->nbSamplesQP);
    }
}

void QPGenerator::precomputeObjectiveAndConstraintsOrientation(){

  int nbUsedPonderations = ponderation_->baseJerkMin.size();

  QconstOr_.resize(nbUsedPonderations);
  pconstOrCoMYaw_.resize(nbUsedPonderations);
  pconstOrRef_.resize(nbUsedPonderations);
  AconstOr_.resize(nbUsedPonderations);

  int N = generalData_->nbSamplesQP;

  MatrixXd idN = MatrixXd::Identity(N,N);

  const LinearDynamics & CoMVelDynamics = robot_->body(COM)->dynamics(velDynamic);
  const LinearDynamics & CoMAccDynamics = robot_->body(COM)->dynamics(accDynamic);

  for (int i = 0; i < nbUsedPonderations; ++i) {
      QconstOr_[i].resize(N,N);
      pconstOrCoMYaw_[i].resize(N,3);
      pconstOrRef_[i].resize(N,N);
      AconstOr_[i].resize(3*N,N);


      QconstOr_[i] = ponderation_->OrientationJerkMin[i] * idN
          + ponderation_->OrientationInstantVelocity[i] * CoMVelDynamics.UT*CoMVelDynamics.U;

      pconstOrRef_[i] = -ponderation_->OrientationInstantVelocity[i] * CoMVelDynamics.UT;
      pconstOrCoMYaw_[i] = -pconstOrRef_[i] * CoMVelDynamics.S;

      AconstOr_[i].block(0,0,N,N) = CoMVelDynamics.U;
      AconstOr_[i].block(N,0,N,N) = CoMAccDynamics.U;
      AconstOr_[i].block(2*N,0,N,N) = idN;
    }
}

void QPGenerator::buildObjectiveOrientation() {

  int nb = ponderation_->activePonderation;
  int N = generalData_->nbSamplesQP;

  const BodyState & CoM = robot_->body(COM)->state();

  solverOrientation_->nbVar(N);
  solverOrientation_->nbCtr(0);

  solverOrientation_->matrix(matrixQ).addTerm(QconstOr_[nb]);

  tmpVec_ = pconstOrCoMYaw_[nb] * CoM.yaw;
  solverOrientation_->matrix(vectorP).addTerm(tmpVec_);

  tmpVec_ = pconstOrRef_[nb] * velRef_->global.yaw;
  solverOrientation_->matrix(vectorP).addTerm(tmpVec_);
}

void QPGenerator::buildConstraintsOrientation(){

  int nb = ponderation_->activePonderation;
  int N = generalData_->nbSamplesQP;
  int nbCtr = solverOrientation_->nbCtr();
  solverOrientation_->addNbCtr(3*N);

  const LinearDynamics & CoMVelDynamics = robot_->body(COM)->dynamics(velDynamic);
  const LinearDynamics & CoMAccDynamics = robot_->body(COM)->dynamics(accDynamic);

  const BodyState & CoM = robot_->body(COM)->state();

  RobotData robotData = robot_->robotData();

  solverOrientation_->matrix(matrixA).addTerm(AconstOr_[nb], nbCtr);


  tmpVec_.resize(3*N);

  tmpVec_.segment(0,N).fill(-robotData.orientationLimit[0]);
  tmpVec_.segment(N,N).fill(-robotData.orientationLimit[1]);
  tmpVec_.segment(2*N,N).fill(-robotData.orientationLimit[2]);

  tmpVec_.segment(0,N) -= CoMVelDynamics.S*CoM.yaw;

  tmpVec_.segment(N,N) -= CoMAccDynamics.S*CoM.yaw;


  solverOrientation_->matrix(vectorBL).addTerm(tmpVec_, nbCtr);


  tmpVec_.segment(0,N).fill(robotData.orientationLimit[0]);
  tmpVec_.segment(N,N).fill(robotData.orientationLimit[1]);
  tmpVec_.segment(2*N,N).fill(robotData.orientationLimit[2]);

  tmpVec_.segment(0,N) -= CoMVelDynamics.S*CoM.yaw;

  tmpVec_.segment(N,N) -= CoMAccDynamics.S*CoM.yaw;

  solverOrientation_->matrix(vectorBU).addTerm(tmpVec_, nbCtr);


  tmpVec_.resize(N);
  tmpVec_.fill(-10e11);
  solverOrientation_->matrix(vectorXL).addTerm(tmpVec_, nbCtr);
  tmpVec_.fill(10e11);
  solverOrientation_->matrix(vectorXU).addTerm(tmpVec_, nbCtr);
}

void QPGenerator::computeWarmStartOrientation(MPCSolution & result){
  if (result.constraintsOrientation.rows()>=solverOrientation_->nbCtr()+solverOrientation_->nbVar()){
      result.initialConstraintsOrientation= result.constraintsOrientation;
      result.initialSolutionOrientation= result.qpSolutionOrientation;
    }else{
      result.initialConstraintsOrientation.setZero((3+1)*generalData_->nbSamplesQP);
      result.initialSolutionOrientation.setZero(generalData_->nbSamplesQP);
    }
}

void QPGenerator::computeOrientationReferenceVector(){

  if (velRef_->global.yaw.rows()!=generalData_->nbSamplesQP){
    velRef_->global.yaw.resize(generalData_->nbSamplesQP);
  }

  for (int i=0;i<generalData_->nbSamplesQP;++i){
     velRef_->global.yaw(i) = velRef_->local.yaw(i);
  }
}

void QPGenerator::computeReferenceVector(const MPCSolution & result){

  computeOrientationMatrices(result);

  if (velRef_->global.x.rows()!=generalData_->nbSamplesQP){
      velRef_->global.x.resize(generalData_->nbSamplesQP);
      velRef_->global.y.resize(generalData_->nbSamplesQP);
    }


  for (int i=0;i<generalData_->nbSamplesQP;++i){

      velRef_->global.x(i) = velRef_->local.x(i)*cosYaw_(i)-velRef_->local.y(i)*sinYaw_(i);
      velRef_->global.y(i) =velRef_->local.x(i)*sinYaw_(i)+velRef_->local.y(i)*cosYaw_(i);
    }
}

void QPGenerator::computeOrientationMatrices(const MPCSolution & result){

  int N = generalData_->nbSamplesQP;

  const LinearDynamics & CoMPosDynamics = robot_->body(COM)->dynamics(posDynamic);
  const BodyState & CoM = robot_->body(COM)->state();

  yaw_ = CoMPosDynamics.S * CoM.yaw + CoMPosDynamics.U * result.qpSolutionOrientation;

  if (cosYaw_.rows()!=N){
    cosYaw_.resize(N);
    sinYaw_.resize(N);
    Rxx_.setZero(N,N);
    Rxy_.setZero(N,N);
    Ryx_.setZero(N,N);
    Ryy_.setZero(N,N);
  }

  for(int i=0;i<N;++i){
    cosYaw_(i)=cos(yaw_(i));
    sinYaw_(i)=sin(yaw_(i));

    Rxx_(i,i)=cosYaw_(i);
    Rxy_(i,i)=-sinYaw_(i);
    Ryx_(i,i)=sinYaw_(i);
    Ryy_(i,i)=cosYaw_(i);
  }

}

