#include "qp-generator.h"
#include "../common/qp-matrix.h"
#include "../common/tools.h"

#include <iostream>
#include <fstream>
#include <cmath>

using namespace MPCWalkgen;
using namespace Zebulon;
using namespace Eigen;

QPGenerator::QPGenerator(QPSolver * solver, Reference * velRef, Reference * posRef,
             Reference *posIntRef, Reference * comRef, Reference * copRef, RigidBodySystem *robot,
             const MPCData *generalData, const RobotData *robotData, const EnvData *envData)
  :solver_(solver)
  ,robot_(robot)
  ,velRef_(velRef)
  ,posRef_(posRef)
  ,posIntRef_(posIntRef)
  ,comRef_(comRef)
  ,copRef_(copRef)
  ,generalData_(generalData)
  ,robotData_(robotData)
  ,envData_(envData)
  ,tmpVec_(1)
  ,tmpVec2_(1)
  ,tmpVec3_(1)
  ,tmpMat_(1,1)
{
}

QPGenerator::~QPGenerator(){}


void QPGenerator::precomputeObjectiveCoP(){

  int nbUsedPonderations = generalData_->ponderation.baseJerkMin.size();

  QCoPconst_.resize(nbUsedPonderations);
  pCoPconstComObjComStateX_.resize(nbUsedPonderations);
  pCoPconstComObjComStateY_.resize(nbUsedPonderations);
  pCoPconstComObjCopRefX_.resize(nbUsedPonderations);
  pCoPconstComObjCopRefY_.resize(nbUsedPonderations);
  pCoPconstComObjBaseStateX_.resize(nbUsedPonderations);
  pCoPconstComObjBaseStateY_.resize(nbUsedPonderations);
  pCoPconstBaseObjComStateX_.resize(nbUsedPonderations);
  pCoPconstBaseObjComStateY_.resize(nbUsedPonderations);

  int N = generalData_->nbSamplesQP;

  const LinearDynamics & CoPDynamicsX = robot_->body(COM)->dynamics(copDynamicX);
  const LinearDynamics & CoPDynamicsY = robot_->body(COM)->dynamics(copDynamicY);
  const LinearDynamics & basePosDynamics = robot_->body(BASE)->dynamics(posDynamic);

  for (int i = 0; i < nbUsedPonderations; ++i){
    QCoPconst_[i].setZero(4*N,4*N);
    pCoPconstComObjComStateX_[i].setZero(N,5);
    pCoPconstComObjComStateY_[i].setZero(N,5);
    pCoPconstComObjCopRefX_[i].setZero(N,N);
    pCoPconstComObjCopRefY_[i].setZero(N,N);
    pCoPconstComObjBaseStateX_[i].setZero(N,5);
    pCoPconstComObjBaseStateY_[i].setZero(N,5);
    pCoPconstBaseObjComStateX_[i].setZero(N,5);
    pCoPconstBaseObjComStateY_[i].setZero(N,5);


    tmpMat_ = generalData_->ponderation.CopCentering[i]*CoPDynamicsX.UT*CoPDynamicsX.U;
    QCoPconst_[i].block(0,0,N,N) = tmpMat_;
    tmpMat_ = generalData_->ponderation.CopCentering[i]*CoPDynamicsY.UT*CoPDynamicsY.U;
    QCoPconst_[i].block(N,N,N,N) = tmpMat_;

    tmpMat_ = -generalData_->ponderation.CopCentering[i]*CoPDynamicsX.UT*basePosDynamics.U;
    QCoPconst_[i].block(0,2*N,N,N) = tmpMat_;
    QCoPconst_[i].block(N,3*N,N,N) = tmpMat_;
    tmpMat_ = -generalData_->ponderation.CopCentering[i]*CoPDynamicsY.UT*basePosDynamics.U;
    tmpMat_.transposeInPlace();
    QCoPconst_[i].block(2*N,0,N,N) = tmpMat_;
    QCoPconst_[i].block(3*N,N,N,N) = tmpMat_;



    tmpMat_ = generalData_->ponderation.CopCentering[i]*CoPDynamicsX.UT*CoPDynamicsX.S;
    pCoPconstComObjComStateX_[i].block(0,0,N,5) = tmpMat_;
    tmpMat_ = generalData_->ponderation.CopCentering[i]*CoPDynamicsY.UT*CoPDynamicsY.S;
    pCoPconstComObjComStateY_[i].block(0,0,N,5) = tmpMat_;

    tmpMat_ = -generalData_->ponderation.CopCentering[i]*basePosDynamics.UT*CoPDynamicsX.S;
    pCoPconstBaseObjComStateX_[i].block(0,0,N,5) = tmpMat_;
    tmpMat_ = -generalData_->ponderation.CopCentering[i]*basePosDynamics.UT*CoPDynamicsY.S;
    pCoPconstBaseObjComStateY_[i].block(0,0,N,5) = tmpMat_;

    tmpMat_ = -generalData_->ponderation.CopCentering[i]*CoPDynamicsX.UT*basePosDynamics.S;
    pCoPconstComObjBaseStateX_[i].block(0,0,N,5) = tmpMat_;
     tmpMat_ = -generalData_->ponderation.CopCentering[i]*CoPDynamicsY.UT*basePosDynamics.S;
    pCoPconstComObjBaseStateY_[i].block(0,0,N,5) = tmpMat_;

    tmpMat_ = -generalData_->ponderation.CopCentering[i]*CoPDynamicsX.UT;
    pCoPconstComObjCopRefX_[i].block(0,0,N,N) = tmpMat_;
    tmpMat_ = -generalData_->ponderation.CopCentering[i]*CoPDynamicsY.UT;
    pCoPconstComObjCopRefY_[i].block(0,0,N,N) = tmpMat_;
  }

}

void QPGenerator::precomputeObjective(){

  precomputeObjectiveCoP();

  int nbUsedPonderations = generalData_->ponderation.baseJerkMin.size();

  Qconst_.resize(nbUsedPonderations);
  pconstComObjComState_.resize(nbUsedPonderations);
  pconstComObjBaseState.resize(nbUsedPonderations);
  pconstBaseObjComState.resize(nbUsedPonderations);
  pconstBaseObjBaseState.resize(nbUsedPonderations);
  pconstBaseObjVelRef_.resize(nbUsedPonderations);
  pconstBaseObjPosRef_.resize(nbUsedPonderations);
  pconstBaseObjPosIntRef_.resize(nbUsedPonderations);
  pconstBaseObjCopRef_.resize(nbUsedPonderations);
  pconstComObjComRef_.resize(nbUsedPonderations);
  pconstBaseObjComRef_.resize(nbUsedPonderations);

  int N = generalData_->nbSamplesQP;

  MatrixXd idN = MatrixXd::Identity(N,N);

  const LinearDynamics & CoMPosDynamics = robot_->body(COM)->dynamics(posDynamic);
  const LinearDynamics & basePosIntDynamics = robot_->body(BASE)->dynamics(posIntDynamic);
  const LinearDynamics & basePosDynamics = robot_->body(BASE)->dynamics(posDynamic);
  const LinearDynamics & baseVelDynamics = robot_->body(BASE)->dynamics(velDynamic);


  for (int i = 0; i < nbUsedPonderations; ++i){
    Qconst_[i].setZero(4*N,4*N);
    pconstComObjComState_[i].setZero(N,5);
    pconstComObjBaseState[i].setZero(N,5);
    pconstBaseObjComState[i].setZero(N,5);
    pconstBaseObjBaseState[i].setZero(N,5);
    pconstBaseObjVelRef_[i].setZero(N,N);
    pconstBaseObjPosRef_[i].setZero(N,N);
    pconstBaseObjPosIntRef_[i].setZero(N,N);
    pconstBaseObjCopRef_[i].setZero(N,N);
    pconstComObjComRef_[i].setZero(N,N);
    pconstBaseObjComRef_[i].setZero(N,N);

    tmpMat_ = generalData_->ponderation.CoMCentering[i]*CoMPosDynamics.UT*CoMPosDynamics.U
        + generalData_->ponderation.CoMJerkMin[i]*idN;
    Qconst_[i].block(0,0,N,N) = tmpMat_;
    Qconst_[i].block(N,N,N,N) = tmpMat_;



    tmpMat_ = -generalData_->ponderation.CoMCentering[i]*CoMPosDynamics.UT*basePosDynamics.U;
    Qconst_[i].block(0,2*N,N,N) = tmpMat_;
    Qconst_[i].block(N,3*N,N,N) = tmpMat_;
    tmpMat_.transposeInPlace();
    Qconst_[i].block(2*N,0,N,N) = tmpMat_;
    Qconst_[i].block(3*N,N,N,N) = tmpMat_;


    tmpMat_ = (generalData_->ponderation.CopCentering[i]+generalData_->ponderation.CoMCentering[i])*basePosDynamics.UT*basePosDynamics.U
        + generalData_->ponderation.baseInstantVelocity[i]*baseVelDynamics.UT*baseVelDynamics.U
        + generalData_->ponderation.basePosition[i]*basePosDynamics.UT*basePosDynamics.U
        + generalData_->ponderation.basePositionInt[i]*basePosIntDynamics.UT*basePosIntDynamics.U
        + generalData_->ponderation.baseJerkMin[i]*idN;
    Qconst_[i].block(2*N,2*N,N,N) = tmpMat_;
    Qconst_[i].block(3*N,3*N,N,N) = tmpMat_;



    tmpMat_ = generalData_->ponderation.CoMCentering[i]*CoMPosDynamics.UT*CoMPosDynamics.S;
    pconstComObjComState_[i].block(0,0,N,5) = tmpMat_;

    tmpMat_ = -generalData_->ponderation.CoMCentering[i]*basePosDynamics.UT*CoMPosDynamics.S;
    pconstBaseObjComState[i].block(0,0,N,5) = tmpMat_;



    tmpMat_ = -generalData_->ponderation.CoMCentering[i]*CoMPosDynamics.UT*basePosDynamics.S;
    pconstComObjBaseState[i].block(0,0,N,5) = tmpMat_;

    tmpMat_ = (generalData_->ponderation.CopCentering[i]+generalData_->ponderation.CoMCentering[i])*basePosDynamics.UT*basePosDynamics.S
        + generalData_->ponderation.baseInstantVelocity[i]*baseVelDynamics.UT*baseVelDynamics.S
        + generalData_->ponderation.basePosition[i]*basePosDynamics.UT*basePosDynamics.S
        + generalData_->ponderation.basePositionInt[i]*basePosIntDynamics.UT*basePosIntDynamics.S        ;
    pconstBaseObjBaseState[i].block(0,0,N,5) = tmpMat_;


    tmpMat_ = -generalData_->ponderation.baseInstantVelocity[i]*baseVelDynamics.UT;
    pconstBaseObjVelRef_[i].block(0,0,N,N) = tmpMat_;

    tmpMat_ = -generalData_->ponderation.basePosition[i]*basePosDynamics.UT;
    pconstBaseObjPosRef_[i].block(0,0,N,N) = tmpMat_;

    tmpMat_ = -generalData_->ponderation.basePositionInt[i]*basePosIntDynamics.UT;
    pconstBaseObjPosIntRef_[i].block(0,0,N,N) = tmpMat_;


    tmpMat_ = generalData_->ponderation.CopCentering[i]*basePosDynamics.UT;
    pconstBaseObjCopRef_[i].block(0,0,N,N) = tmpMat_;

    tmpMat_ = -generalData_->ponderation.CoMCentering[i]*CoMPosDynamics.UT;
    pconstComObjComRef_[i].block(0,0,N,N) = tmpMat_;

    tmpMat_ = generalData_->ponderation.CoMCentering[i]*basePosDynamics.UT;
    pconstBaseObjComRef_[i].block(0,0,N,N) = tmpMat_;


  }
}

void QPGenerator::computePartOfVectorP(const Eigen::MatrixXd & precomputedMatrix,
                                       const Eigen::VectorXd & state,
                                       int pos,
                                       bool setTerm){
  tmpVec_ = precomputedMatrix * state;

  if (setTerm){
    solver_->vector(vectorP).setTerm(tmpVec_,pos);
  }else{
    solver_->vector(vectorP).addTerm(tmpVec_,pos);
  }

}

void QPGenerator::computePartOfVectorP(const Eigen::MatrixXd & precomputedMatrix,
                                       const BodyState & state,
                                       int pos,
                                       bool setTerm){
  computePartOfVectorP(precomputedMatrix, state.x, pos, setTerm);
  computePartOfVectorP(precomputedMatrix, state.y, pos+generalData_->nbSamplesQP, setTerm);
}


void QPGenerator::computePartOfVectorP(const Eigen::MatrixXd & precomputedMatrix,
                                       const Reference::Frame & state,
                                       int pos,
                                       bool setTerm){
  computePartOfVectorP(precomputedMatrix, state.x, pos, setTerm);
  computePartOfVectorP(precomputedMatrix, state.y, pos+generalData_->nbSamplesQP, setTerm);
}

void QPGenerator::buildObjective() {

  int nb = generalData_->ponderation.activePonderation;
  int N = generalData_->nbSamplesQP;

  const BodyState & CoM = robot_->body(COM)->state();
  const BodyState & base = robot_->body(BASE)->state();

  solver_->nbVar(4*N);

  solver_->matrix(matrixQ).setTerm(Qconst_[nb]);
  solver_->matrix(matrixQ).addTerm(QCoPconst_[nb]);

  computePartOfVectorP(pconstComObjComState_[nb], CoM, 0, true);
  computePartOfVectorP(pCoPconstComObjComStateX_[nb], CoM.x, 0);
  computePartOfVectorP(pCoPconstComObjComStateY_[nb], CoM.y, N);

  computePartOfVectorP(pconstBaseObjComState[nb], CoM, 2*N);
  computePartOfVectorP(pCoPconstBaseObjComStateX_[nb], CoM.x, 2*N);
  computePartOfVectorP(pCoPconstBaseObjComStateY_[nb], CoM.y, 3*N);

  computePartOfVectorP(pconstComObjBaseState[nb], base, 0);
  computePartOfVectorP(pCoPconstComObjBaseStateX_[nb], base.x, 0);
  computePartOfVectorP(pCoPconstComObjBaseStateY_[nb], base.y, N);

  computePartOfVectorP(pconstBaseObjBaseState[nb], base, 2*N);

  computePartOfVectorP(pconstBaseObjVelRef_[nb], velRef_->global, 2*N);
  computePartOfVectorP(pconstBaseObjPosRef_[nb], posRef_->global, 2*N);
  computePartOfVectorP(pconstBaseObjPosIntRef_[nb], posIntRef_->global, 2*N);

  computePartOfVectorP(pconstBaseObjCopRef_[nb], copRef_->global, 2*N);
  computePartOfVectorP(pCoPconstComObjCopRefX_[nb], copRef_->global.x, 0);
  computePartOfVectorP(pCoPconstComObjCopRefY_[nb], copRef_->global.y, N);

  computePartOfVectorP(pconstComObjComRef_[nb], comRef_->global, 0);
  computePartOfVectorP(pconstBaseObjComRef_[nb], comRef_->global, 2*N);


}

void QPGenerator::buildConstraintsCoP(){

  int N = generalData_->nbSamplesQP;
  const LinearDynamics & CoPDynamicsX = robot_->body(COM)->dynamics(copDynamicX);
  const LinearDynamics & CoPDynamicsY = robot_->body(COM)->dynamics(copDynamicY);
  const LinearDynamics & basePosDynamics = robot_->body(BASE)->dynamics(posDynamic);
  const BodyState & CoM = robot_->body(COM)->state();
  const BodyState & base = robot_->body(BASE)->state();

  double factor = 2*robotData_->copLimitY/robotData_->copLimitX;

  tmpMat_ = -(Rxx_.asDiagonal()*CoPDynamicsX.U);
  solver_->matrix(matrixA).setTerm(tmpMat_, 0, 0);
  solver_->matrix(matrixA).setTerm(tmpMat_, N, 0);
  solver_->matrix(matrixA).setTerm(-tmpMat_, 2*N, 0);

  tmpMat_ = -(Rxy_.asDiagonal()*CoPDynamicsY.U);
  solver_->matrix(matrixA).setTerm(tmpMat_, 0, N);
  solver_->matrix(matrixA).setTerm(tmpMat_, N, N);
  solver_->matrix(matrixA).setTerm(-tmpMat_, 2*N, N);


  tmpMat_ = factor*(Ryx_.asDiagonal()*CoPDynamicsX.U);
  solver_->matrix(matrixA).addTerm(tmpMat_, 0, 0);
  solver_->matrix(matrixA).addTerm(-tmpMat_, N, 0);

  tmpMat_ = factor*(Ryy_.asDiagonal()*CoPDynamicsY.U);
  solver_->matrix(matrixA).addTerm(tmpMat_, 0, N);
  solver_->matrix(matrixA).addTerm(-tmpMat_, N, N);



  tmpMat_ = Rxx_.asDiagonal()*basePosDynamics.U;
  solver_->matrix(matrixA).setTerm(tmpMat_, 0, 2*N);
  solver_->matrix(matrixA).setTerm(tmpMat_, N, 2*N);
  solver_->matrix(matrixA).setTerm(-tmpMat_, 2*N, 2*N);

  tmpMat_ = Rxy_.asDiagonal()*basePosDynamics.U;
  solver_->matrix(matrixA).setTerm(tmpMat_, 0, 3*N);
  solver_->matrix(matrixA).setTerm(tmpMat_, N, 3*N);
  solver_->matrix(matrixA).setTerm(-tmpMat_, 2*N, 3*N);



  tmpMat_ = -factor*(Ryx_.asDiagonal()*basePosDynamics.U);
  solver_->matrix(matrixA).addTerm(tmpMat_, 0, 2*N);
  solver_->matrix(matrixA).addTerm(-tmpMat_, N, 2*N);

  tmpMat_ = -factor*(Ryy_.asDiagonal()*basePosDynamics.U);
  solver_->matrix(matrixA).addTerm(tmpMat_, 0, 3*N);
  solver_->matrix(matrixA).addTerm(-tmpMat_, N, 3*N);


  tmpVec_.resize(3*N);
  tmpVec_.fill(-10e11);
  solver_->vector(vectorBL).setTerm(tmpVec_, 0);




  tmpVec_.segment(0,2*N).fill(robotData_->copLimitY/2+robotData_->deltaComXLocal);
  tmpVec_.segment(2*N,N).fill(robotData_->copLimitY/2-robotData_->deltaComXLocal);

  tmpVec_.segment(0,N) += Rxx_.asDiagonal()*(CoPDynamicsX.S * CoM.x - basePosDynamics.S*base.x)
                       + factor*(Ryy_.asDiagonal()*(-CoPDynamicsY.S* CoM.y + basePosDynamics.S*base.y));
  tmpVec_.segment(N,N) += Rxx_.asDiagonal()*(CoPDynamicsX.S * CoM.x - basePosDynamics.S*base.x)
                       + factor*(Ryy_.asDiagonal()*( CoPDynamicsY.S* CoM.y - basePosDynamics.S*base.y));
  tmpVec_.segment(2*N,N) += Rxx_.asDiagonal()*(-CoPDynamicsX.S*CoM.x + basePosDynamics.S*base.x);

  tmpVec_.segment(0,N) += Rxy_.asDiagonal()*(CoPDynamicsY.S * CoM.y - basePosDynamics.S*base.y)
                       + factor*(Ryx_.asDiagonal()*(-CoPDynamicsX.S* CoM.x + basePosDynamics.S*base.x));
  tmpVec_.segment(N,N) += Rxy_.asDiagonal()*(CoPDynamicsY.S * CoM.y - basePosDynamics.S*base.y)
                       + factor*(Ryx_.asDiagonal()*( CoPDynamicsX.S* CoM.x - basePosDynamics.S*base.x));
  tmpVec_.segment(2*N,N) += Rxy_.asDiagonal()*(-CoPDynamicsY.S*CoM.y + basePosDynamics.S*base.y);


  solver_->vector(vectorBU).setTerm(tmpVec_, 0);

}

void QPGenerator::buildConstraintsCoM(){

  int N = generalData_->nbSamplesQP;
  const LinearDynamics & CoMPosDynamics = robot_->body(COM)->dynamics(posDynamic);
  const LinearDynamics & basePosDynamics = robot_->body(BASE)->dynamics(posDynamic);
  const BodyState & CoM = robot_->body(COM)->state();
  const BodyState & base = robot_->body(BASE)->state();

  tmpMat_ = Rxx_.asDiagonal()*CoMPosDynamics.U;
  solver_->matrix(matrixA).setTerm(tmpMat_, 3*N, 0);
  tmpMat_ = Rxy_.asDiagonal()*CoMPosDynamics.U;
  solver_->matrix(matrixA).setTerm(tmpMat_, 3*N, N);


  tmpMat_ = Ryx_.asDiagonal()*CoMPosDynamics.U;
  solver_->matrix(matrixA).setTerm(tmpMat_, 4*N, 0);
  tmpMat_ = Ryy_.asDiagonal()*CoMPosDynamics.U;
  solver_->matrix(matrixA).setTerm(tmpMat_, 4*N, N);

  tmpMat_ = -(Rxx_.asDiagonal()*basePosDynamics.U);
  solver_->matrix(matrixA).setTerm(tmpMat_, 3*N, 2*N);
  tmpMat_ = -(Rxy_.asDiagonal()*basePosDynamics.U);
  solver_->matrix(matrixA).setTerm(tmpMat_, 3*N, 3*N);

  tmpMat_ = -(Ryx_.asDiagonal()*basePosDynamics.U);
  solver_->matrix(matrixA).setTerm(tmpMat_, 4*N, 2*N);
  tmpMat_ = -(Ryy_.asDiagonal()*basePosDynamics.U);
  solver_->matrix(matrixA).setTerm(tmpMat_, 4*N, 3*N);




  tmpVec3_.resize(2*N);

  tmpVec2_ = -CoMPosDynamics.S*CoM.x + basePosDynamics.S*base.x;
  tmpVec3_.segment(0,N) = Rxx_.asDiagonal()*tmpVec2_;
  tmpVec3_.segment(N,N) = Ryx_.asDiagonal()*tmpVec2_;

  tmpVec2_ = -CoMPosDynamics.S*CoM.y + basePosDynamics.S*base.y;
  tmpVec3_.segment(0,N) += Rxy_.asDiagonal()*tmpVec2_;
  tmpVec3_.segment(N,N) += Ryy_.asDiagonal()*tmpVec2_;

  tmpVec_.resize(2*N);

  tmpVec_.segment(0,N).fill(-robotData_->comLimitX);
  tmpVec_.segment(N,N).fill(-robotData_->comLimitY);
  solver_->vector(vectorBL).setTerm(tmpVec_+tmpVec3_, 3*N);


  tmpVec_.segment(0,N).fill(robotData_->comLimitX);
  tmpVec_.segment(N,N).fill(robotData_->comLimitY);
  solver_->vector(vectorBU).setTerm(tmpVec_+tmpVec3_, 3*N);

}

//The base must be out of each circular obstacle, linearized by a tangent plane
void QPGenerator::buildConstraintsBasePosition(){
  const int size = envData_->nbObstacle;
  const int N = generalData_->nbSamplesQP;
  const LinearDynamics & basePosDynamics = robot_->body(BASE)->dynamics(posDynamic);
  const BodyState & base = robot_->body(BASE)->state();

  tmpMat_.setZero(N*size, 2*N);
  tmpVec_.setZero(N*size);
  tmpVec3_.resize(N);
  tmpVec2_.resize(N*size);
  tmpVec2_.fill(-10e11);
  for(int j=0; j<size; ++j)
  {
    for(int i=0; i<N; ++i)
    {
      const double dist =
     std::sqrt(pow2(envData_->obstaclePositionX(j)-envData_->obstacleLinearizationPointX(i))
              +pow2(envData_->obstaclePositionY(j)-envData_->obstacleLinearizationPointY(i)));

      const double FX = envData_->obstaclePositionX(j) + (envData_->obstacleRadius(j)/dist)
                     * (envData_->obstacleLinearizationPointX(i) - envData_->obstaclePositionX(j));
      const double FY = envData_->obstaclePositionY(j) + (envData_->obstacleRadius(j)/dist)
                     * (envData_->obstacleLinearizationPointY(i) - envData_->obstaclePositionY(j));

      const double FPX = envData_->obstaclePositionX(j) - FX;
      const double FPY = envData_->obstaclePositionY(j) - FY;

      tmpMat_(j*N+i, i)   = FPX;
      tmpMat_(j*N+i, i+N) = FPY;

      tmpVec_(j*N+i) = FX*FPX+FY*FPY;
    }

    tmpVec_.segment(j*N, N) = tmpVec_.segment(j*N, N)-tmpMat_.block(j*N, 0, N, N)*basePosDynamics.S*base.x;
    tmpVec_.segment(j*N, N) = tmpVec_.segment(j*N, N)-tmpMat_.block(j*N, N, N, N)*basePosDynamics.S*base.y;
    tmpMat_.block(j*N, 0, N, N) = tmpMat_.block(j*N, 0, N, N)*basePosDynamics.U;
    tmpMat_.block(j*N, N, N, N) = tmpMat_.block(j*N, N, N, N)*basePosDynamics.U;

  }


  solver_->matrix(matrixA).setTerm(tmpMat_, 9*N, 2*N);
  solver_->vector(vectorBL).setTerm(tmpVec2_, 9*N);
  solver_->vector(vectorBU).setTerm(tmpVec_, 9*N);
}

void QPGenerator::buildConstraintsBaseVelocity(){

  int N = generalData_->nbSamplesQP;
  const LinearDynamics & baseVelDynamics = robot_->body(BASE)->dynamics(velDynamic);
  const BodyState & base = robot_->body(BASE)->state();

  solver_->matrix(matrixA).setTerm(baseVelDynamics.U, 5*N, 2*N);
  solver_->matrix(matrixA).setTerm(baseVelDynamics.U, 6*N, 3*N);

  tmpVec3_.resize(2*N);
  tmpVec3_.segment(0,N) = -baseVelDynamics.S*base.x;
  tmpVec3_.segment(N,N) = -baseVelDynamics.S*base.y;


  tmpVec_.resize(2*N);

  tmpVec_.fill(-robotData_->baseLimit[0]);
  solver_->vector(vectorBL).setTerm(tmpVec_+tmpVec3_, 5*N);

  tmpVec_.fill(robotData_->baseLimit[0]);
  solver_->vector(vectorBU).setTerm(tmpVec_+tmpVec3_, 5*N);

}

void QPGenerator::buildConstraintsBaseAcceleration(){

  int N = generalData_->nbSamplesQP;
  const LinearDynamics & baseAccDynamics = robot_->body(BASE)->dynamics(accDynamic);
  const BodyState & base = robot_->body(BASE)->state();

  solver_->matrix(matrixA).setTerm(baseAccDynamics.U, 7*N, 2*N);
  solver_->matrix(matrixA).setTerm(baseAccDynamics.U, 8*N, 3*N);

  tmpVec3_.resize(2*N);
  tmpVec3_.segment(0,N) = -baseAccDynamics.S*base.x;
  tmpVec3_.segment(N,N) = -baseAccDynamics.S*base.y;

  tmpVec_.resize(2*N);
  tmpVec_.fill(-robotData_->baseLimit[1]);
  solver_->vector(vectorBL).setTerm(tmpVec_+tmpVec3_, 7*N);


  tmpVec_.fill(robotData_->baseLimit[1]);
  solver_->vector(vectorBU).setTerm(tmpVec_+tmpVec3_, 7*N);

}

void QPGenerator::buildConstraintsBaseJerk(){

  int N = generalData_->nbSamplesQP;

  tmpVec_.resize(4*N);


  tmpVec_.segment(0,2*N).fill(-10e11);
  tmpVec_.segment(2*N,2*N).fill(-robotData_->baseLimit[2]);

  solver_->vector(vectorXL).setTerm(tmpVec_, 0);


  tmpVec_.segment(0,2*N).fill(10e11);
  tmpVec_.segment(2*N,2*N).fill(robotData_->baseLimit[2]);

  solver_->vector(vectorXU).setTerm(tmpVec_, 0);

}

void QPGenerator::buildConstraints(){

  solver_->nbCtr(generalData_->QPNbConstraints);

  solver_->nbVar(generalData_->QPNbVariables);

  buildConstraintsCoP();
  buildConstraintsCoM();
  buildConstraintsBasePosition();
  buildConstraintsBaseVelocity();
  buildConstraintsBaseAcceleration();
  buildConstraintsBaseJerk();

}

void QPGenerator::computeWarmStart(GlobalSolution & result){
  if (result.constraints.rows()>=solver_->nbCtr()+solver_->nbVar()){
      result.initialConstraints= result.constraints;
      result.initialSolution= result.qpSolution;
    }else{
      result.initialConstraints.setZero(generalData_->QPNbConstraints
                                      + generalData_->QPNbVariables);
      result.initialSolution.setZero(generalData_->QPNbVariables);
    }
}

void QPGenerator::computeReferenceVector(const GlobalSolution & result){

  computeOrientationMatrices(result);

  if (velRef_->global.x.rows()!=generalData_->nbSamplesQP){
    velRef_->global.x.setZero(generalData_->nbSamplesQP);
    velRef_->global.y.setZero(generalData_->nbSamplesQP);
  }

  if (comRef_->global.x.rows()!=generalData_->nbSamplesQP){
    comRef_->global.x.setZero(generalData_->nbSamplesQP);
    comRef_->global.y.setZero(generalData_->nbSamplesQP);
  }

  if (copRef_->global.x.rows()!=generalData_->nbSamplesQP){
    copRef_->global.x.setZero(generalData_->nbSamplesQP);
    copRef_->global.y.setZero(generalData_->nbSamplesQP);
  }

  for (int i=0;i<generalData_->nbSamplesQP;++i){

    velRef_->global.x(i) += velRef_->local.x(i)*cosYaw_(i)-velRef_->local.y(i)*sinYaw_(i);
    velRef_->global.y(i) += velRef_->local.x(i)*sinYaw_(i)+velRef_->local.y(i)*cosYaw_(i);

    comRef_->global.x(i) = comRef_->local.x(i)*cosYaw_(i)-comRef_->local.y(i)*sinYaw_(i);
    comRef_->global.y(i) = comRef_->local.x(i)*sinYaw_(i)+comRef_->local.y(i)*cosYaw_(i);

    copRef_->global.x(i) = copRef_->local.x(i)*cosYaw_(i)-copRef_->local.y(i)*sinYaw_(i);
    copRef_->global.y(i) = copRef_->local.x(i)*sinYaw_(i)+copRef_->local.y(i)*cosYaw_(i);
  }
}

void QPGenerator::computeOrientationMatrices(const GlobalSolution & result){

  int N = generalData_->nbSamplesQP;

  const LinearDynamics & CoMPosDynamics = robot_->body(COM)->dynamics(posDynamic);
  const BodyState & CoM = robot_->body(COM)->state();

  yaw_ = CoMPosDynamics.S * CoM.yaw + CoMPosDynamics.U * result.qpSolutionOrientation;

  if (cosYaw_.rows()!=N){
    cosYaw_.resize(N);
    sinYaw_.resize(N);
    Rxx_.resize(N);
    Rxy_.resize(N);
    Ryx_.resize(N);
    Ryy_.resize(N);
  }

  for(int i=0;i<N;++i){
    cosYaw_(i)=cos(yaw_(i));
    sinYaw_(i)=sin(yaw_(i));

    Rxx_(i)=cosYaw_(i);
    Rxy_(i)=-sinYaw_(i);
    Ryx_(i)=sinYaw_(i);
    Ryy_(i)=cosYaw_(i);
  }

}

