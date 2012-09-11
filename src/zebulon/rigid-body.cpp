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
    case posIntDynamic:
      return posInt_vec_;
    case posDynamic:
      return pos_vec_;
    case velDynamic:
      return vel_vec_;
    case accDynamic:
      return acc_vec_;
    case jerkDynamic:
      return jerk_vec_;
    case copDynamicX:
      return copX_vec_;
    case copDynamicY:
      return copY_vec_;
    case interpolationPosInt:
      return posIntInterpol_;
    case interpolationPos:
      return posInterpol_;
    case interpolationVel:
      return velInterpol_;
    case interpolationAcc:
      return accInterpol_;
    case interpolationCoPX:
      return copXInterpol_;
    default:
      return copYInterpol_;
    }
}

void RigidBody::computeQPDynamics(){

  computeDynamicsMatrices(posInt_vec_, generalData_->QPSamplingPeriod,
                          generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, posIntDynamic);
  computeDynamicsMatrices(pos_vec_, generalData_->QPSamplingPeriod,
                          generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, posDynamic);
  computeDynamicsMatrices(vel_vec_, generalData_->QPSamplingPeriod,
                          generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, velDynamic);
  computeDynamicsMatrices(acc_vec_, generalData_->QPSamplingPeriod,
                          generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, accDynamic);
  computeDynamicsMatrices(jerk_vec_, generalData_->QPSamplingPeriod,
                          generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, jerkDynamic);

}

void RigidBody::computeInterpolationDynamics(){

  int nbSamplingSim = generalData_->nbSamplesControl();

  computeDynamicsMatrices(posIntInterpol_, generalData_->actuationSamplingPeriod,
                          generalData_->actuationSamplingPeriod, nbSamplingSim, posIntDynamic);

  computeDynamicsMatrices(posInterpol_, generalData_->actuationSamplingPeriod,
                          generalData_->actuationSamplingPeriod, nbSamplingSim, posDynamic);

  computeDynamicsMatrices(velInterpol_, generalData_->actuationSamplingPeriod,
                          generalData_->actuationSamplingPeriod, nbSamplingSim, velDynamic);

  computeDynamicsMatrices(accInterpol_, generalData_->actuationSamplingPeriod,
                          generalData_->actuationSamplingPeriod, nbSamplingSim, accDynamic);

}

void RigidBody::computeQPDynamicsCoP(){

  computeDynamicsMatrices(copX_vec_, generalData_->QPSamplingPeriod,
                          generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, copDynamicX);
  computeDynamicsMatrices(copY_vec_, generalData_->QPSamplingPeriod,
                          generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, copDynamicY);

}

void RigidBody::computeInterpolationDynamicsCoP(){

  int nbSamplingSim = generalData_->nbSamplesControl();

  computeDynamicsMatrices(copXInterpol_, generalData_->actuationSamplingPeriod,
                          generalData_->actuationSamplingPeriod, nbSamplingSim, copDynamicX);

  computeDynamicsMatrices(copYInterpol_, generalData_->actuationSamplingPeriod,
                          generalData_->actuationSamplingPeriod, nbSamplingSim, copDynamicY);

}
