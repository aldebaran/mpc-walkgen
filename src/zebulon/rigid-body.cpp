#include "rigid-body.h"

using namespace MPCWalkgen;
using namespace Zebulon;

RigidBody::RigidBody(const MPCData * generalData,
                     const RobotData * robotData, const Interpolation * interpolation)
  :generalData_(generalData)
  ,robotData_(robotData)
  ,interpolation_(interpolation)
{}

RigidBody::~RigidBody()
{}

const LinearDynamics & RigidBody::dynamics(DynamicMatrixType type) const{
  switch (type){
    case posDynamic:
      return pos_vec_;
    case velDynamic:
      return vel_vec_;
    case accDynamic:
      return acc_vec_;
    case jerkDynamic:
      return jerk_vec_;
    case copDynamic:
      return cop_vec_;
    case interpolationPos:
      return posInterpol_;
    case interpolationVel:
      return velInterpol_;
    case interpolationAcc:
      return accInterpol_;
    default:
      return copInterpol_;
    }
}

void RigidBody::computeDynamics(){

  computeDynamicsMatrices(pos_vec_, generalData_->QPSamplingPeriod,
                          generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, posDynamic);
  computeDynamicsMatrices(vel_vec_, generalData_->QPSamplingPeriod,
                          generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, velDynamic);
  computeDynamicsMatrices(acc_vec_, generalData_->QPSamplingPeriod,
                          generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, accDynamic);
  computeDynamicsMatrices(jerk_vec_, generalData_->QPSamplingPeriod,
                          generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, jerkDynamic);
  computeDynamicsMatrices(cop_vec_, generalData_->QPSamplingPeriod,
                          generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, copDynamic);

  int nbSamplingSim = generalData_->nbSamplesControl();
  computeDynamicsMatrices(posInterpol_, generalData_->actuationSamplingPeriod,
                          generalData_->actuationSamplingPeriod, nbSamplingSim, posDynamic);

  computeDynamicsMatrices(velInterpol_, generalData_->actuationSamplingPeriod,
                          generalData_->actuationSamplingPeriod, nbSamplingSim, velDynamic);

  computeDynamicsMatrices(accInterpol_, generalData_->actuationSamplingPeriod,
                          generalData_->actuationSamplingPeriod, nbSamplingSim, accDynamic);

  computeDynamicsMatrices(copInterpol_, generalData_->actuationSamplingPeriod,
                          generalData_->actuationSamplingPeriod, nbSamplingSim, copDynamic);
}

