#include <mpc-walkgen/rigid-bodies/foot-body.h>
#include <mpc-walkgen/tools.h>
using namespace MPCWalkgen;
using namespace Eigen;


FootBody::FootBody(const MPCData * generalData,
		const RobotData * robotData,
		const Interpolation * interpolation, Foot type)
	:RigidBody(generalData, robotData, interpolation)
	,footType_(type)
{}

FootBody::~FootBody(){}

void FootBody::interpolate(MPCSolution & result, double currentTime, const VelReference & /*velRef*/){

	BodyState nextSupportFootState;

	double Txy=1;
	double Tz=1;
	int nbSampling = generalData_->nbIterationSimulation();
	if (result.supportStates_vec[0].phase == SS){

		int nbStepsPreviewed = result.supportStates_vec.back().stepNumber;

		Txy = result.supportStates_vec[0].startTime+generalData_->stepPeriod-currentTime;

		if (result.supportStates_vec[1].inTransitionalDS){
			nextSupportFootState.x(0)=state_.x(0);
			nextSupportFootState.y(0)=state_.y(0);
			nextSupportFootState.yaw(0)=state_.yaw(0);
		}else{
			int nbPreviewedSteps = result.supportStates_vec.back().stepNumber;

			if (nbPreviewedSteps>0){
				nextSupportFootState.x(0)=result.solution(2*generalData_->nbSamplesQP);
				nextSupportFootState.y(0)=result.solution(2*generalData_->nbSamplesQP+nbStepsPreviewed);
				nextSupportFootState.yaw(0)=result.supportOrientations_vec[0];
			}else{
				nextSupportFootState.x(0)=state_.x(0);
				nextSupportFootState.y(0)=state_.y(0);
				nextSupportFootState.yaw(0)=state_.yaw(0);
			}
		}

		if (Txy-generalData_->stepPeriod/2>generalData_->simSamplingPeriod){
			nextSupportFootState.z(0) = robotData_->freeFlyingFootMaxHeight;
			Tz = result.supportStates_vec[0].startTime+generalData_->stepPeriod/2-currentTime;
		}else{
			Tz = Txy;
		}

	}

	computeFootInterpolationByPolynomial(result, X, nbSampling,
			state_.x,
			Txy, nextSupportFootState.x);
	computeFootInterpolationByPolynomial(result, Y, nbSampling,
			state_.y,
			Txy, nextSupportFootState.y);
	computeFootInterpolationByPolynomial(result, Z, nbSampling,
			state_.z,
			Tz, nextSupportFootState.z);
	computeFootInterpolationByPolynomial(result, Yaw, nbSampling,
			state_.yaw,
			Txy, nextSupportFootState.yaw);

}

void FootBody::computeOneDynamicMatrices(DynamicMatrix & dyn,
double /*S*/, double /*T*/, int N, DynamicMatrixType type){
	dyn.S.setZero(N,3);
	dyn.U.setZero(N,N);
	dyn.UT.setZero(N,N);
	dyn.UInv.setZero(N,N);
	dyn.UInvT.setZero(N,N);


	switch (type){
		case posDynamic:
		break;

		case velDynamic:
		break;

		case accDynamic:
		break;

		case copDynamic:
		break;

		default:
		break;
	}
}



VectorXd & FootBody::getFootVector(MPCSolution & solution, Axis axis, unsigned derivative) {
	MPCSolution::State & currentState = solution.state_vec[derivative];
	if (footType_==LEFT){
		switch(axis){
			case X:
				return currentState.leftFootTrajX_;
			case Y:
				return currentState.leftFootTrajY_;
			case Z:
				return currentState.leftFootTrajZ_;
			default:
				return currentState.leftFootTrajYaw_;
		}
	}else{
		switch(axis){
			case X:
				return currentState.rightFootTrajX_;
			case Y:
				return currentState.rightFootTrajY_;
			case Z:
				return currentState.rightFootTrajZ_;
			default:
				return currentState.rightFootTrajYaw_;
		}
	}
}

void FootBody::computeFootInterpolationByPolynomial(MPCSolution & result, Axis axis, int nbSampling,
		const Eigen::Vector3d & FootCurrentState,
		double T, const Eigen::Vector3d & nextSupportFootState){

	VectorXd & FootTrajState = getFootVector(result, axis, 0);
	VectorXd & FootTrajVel   = getFootVector(result, axis, 1);
	VectorXd & FootTrajAcc   = getFootVector(result, axis, 2);

	if (FootTrajState.rows() != nbSampling){
		FootTrajState.resize(nbSampling);
		FootTrajVel.resize(nbSampling);
		FootTrajAcc.resize(nbSampling);
	}

	if (result.supportStates_vec[0].foot==footType_){
		FootTrajState.fill(FootCurrentState(0));
		FootTrajVel.fill(0);
		FootTrajAcc.fill(0);

	}else{

		if (result.supportStates_vec[0].phase == DS){
			FootTrajState.fill(FootCurrentState(0));
			FootTrajVel.fill(0);
			FootTrajAcc.fill(0);

		}else{
			Eigen::Matrix<double,6,1> factor;
			interpolation_->computePolynomialNormalisedFactors(factor, FootCurrentState, nextSupportFootState, T);
			for(int i=0;i<nbSampling;++i){
				double ti = (i+1)*generalData_->simSamplingPeriod;

				FootTrajState(i) = p(factor, ti/T);
				FootTrajVel(i)   = dp(factor, ti/T)/T;
				FootTrajAcc(i)   = ddp(factor, ti/T)/(T*T);

			}
		}


	}
}
