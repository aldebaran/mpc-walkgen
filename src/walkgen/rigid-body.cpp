#include <mpc-walkgen/rigid-body.h>

using namespace MPCWalkgen;


RigidBody::RigidBody(const MPCData * generalData,
		const RobotData * robotData, const Interpolation * interpolation)
	:generalData_(generalData)
	,robotData_(robotData)
	,interpolation_(interpolation)
{}

RigidBody::~RigidBody(){}


const DynamicMatrix & RigidBody::dynamic(DynamicMatrixType type) const{
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

//TODO: firstSamplingPeriod
void RigidBody::firstIterationDuration(double firstIterationDuration){
	matrixNumber_ = (int)round(firstIterationDuration / generalData_->MPCSamplingPeriod)-1;
}


void RigidBody::computeDynamics(){

	int vecSize = generalData_->nbIterationFeedback();

	pos_vec_.resize(vecSize);
	vel_vec_.resize(vecSize);
	acc_vec_.resize(vecSize);
	jerk_vec_.resize(vecSize);
	cop_vec_.resize(vecSize);

	for(int k=0;k<vecSize ;++k){
		double S = generalData_->MPCSamplingPeriod * (k+1);
		computeOneDynamicMatrices(pos_vec_[k], S,
				generalData_->QPSamplingPeriod, generalData_->QPNbSamplings, posDynamic);
		computeOneDynamicMatrices(vel_vec_[k], S,
				generalData_->QPSamplingPeriod, generalData_->QPNbSamplings, velDynamic);
		computeOneDynamicMatrices(acc_vec_[k], S,
				generalData_->QPSamplingPeriod, generalData_->QPNbSamplings, accDynamic);
		computeOneDynamicMatrices(jerk_vec_[k], S,
				generalData_->QPSamplingPeriod, generalData_->QPNbSamplings, jerkDynamic);
		computeOneDynamicMatrices(cop_vec_[k], S,
				generalData_->QPSamplingPeriod, generalData_->QPNbSamplings, copDynamic);

	}

	int nbSamplingSim = generalData_->nbIterationSimulation();
	computeOneDynamicMatrices(posInterpol_, generalData_->simSamplingPeriod,
			generalData_->simSamplingPeriod, nbSamplingSim, posDynamic);

	computeOneDynamicMatrices(velInterpol_, generalData_->simSamplingPeriod,
			generalData_->simSamplingPeriod, nbSamplingSim, velDynamic);

	computeOneDynamicMatrices(accInterpol_, generalData_->simSamplingPeriod,
			generalData_->simSamplingPeriod, nbSamplingSim, accDynamic);

	computeOneDynamicMatrices(copInterpol_, generalData_->simSamplingPeriod,
			generalData_->simSamplingPeriod, nbSamplingSim, copDynamic);



}
