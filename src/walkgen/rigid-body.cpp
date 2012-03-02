#include <mpc-walkgen/rigid-body.h>

using namespace MPCWalkgen;


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
			return pos_vec_[matrixNumber_];
		case velDynamic:
			return vel_vec_[matrixNumber_];
		case accDynamic:
			return acc_vec_[matrixNumber_];
		case jerkDynamic:
			return jerk_vec_[matrixNumber_];
		case copDynamic:
			return cop_vec_[matrixNumber_];
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

void RigidBody::setDynamics(double firstSamplingPeriod){
	matrixNumber_ = (int)round(firstSamplingPeriod / generalData_->MPCSamplingPeriod)-1;
}

void RigidBody::computeDynamics(){

	int vecSize = generalData_->nbFeedbackSamplesStandard();

	pos_vec_.resize(vecSize);
	vel_vec_.resize(vecSize);
	acc_vec_.resize(vecSize);
	jerk_vec_.resize(vecSize);
	cop_vec_.resize(vecSize);

	for (int k=0; k<vecSize; ++k) {
		double S = generalData_->MPCSamplingPeriod * (k+1);
		computeDynamicsMatrices(pos_vec_[k], S,
				generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, posDynamic);
		computeDynamicsMatrices(vel_vec_[k], S,
				generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, velDynamic);
		computeDynamicsMatrices(acc_vec_[k], S,
				generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, accDynamic);
		computeDynamicsMatrices(jerk_vec_[k], S,
				generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, jerkDynamic);
		computeDynamicsMatrices(cop_vec_[k], S,
				generalData_->QPSamplingPeriod, generalData_->nbSamplesQP, copDynamic);

	}

	int nbSamplingSim = generalData_->nbIterationSimulation();
	computeDynamicsMatrices(posInterpol_, generalData_->actuationSamplingPeriod,
			generalData_->actuationSamplingPeriod, nbSamplingSim, posDynamic);

	computeDynamicsMatrices(velInterpol_, generalData_->actuationSamplingPeriod,
			generalData_->actuationSamplingPeriod, nbSamplingSim, velDynamic);

	computeDynamicsMatrices(accInterpol_, generalData_->actuationSamplingPeriod,
			generalData_->actuationSamplingPeriod, nbSamplingSim, accDynamic);

	computeDynamicsMatrices(copInterpol_, generalData_->actuationSamplingPeriod,
			generalData_->actuationSamplingPeriod, nbSamplingSim, copDynamic);
}

//void RigidBody::simulateForward() {
//
//}
