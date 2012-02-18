#include <mpc-walkgen/rigid-body-system.h>
#include <mpc-walkgen/rigid-bodies/com-body.h>
#include <mpc-walkgen/rigid-bodies/foot-body.h>

using namespace MPCWalkgen;
using namespace Eigen;

RigidBodySystem::RigidBodySystem(const MPCData * generalData, RobotData * robotData, const Interpolation * interpolation)
:generalData_(generalData)
,robotData_(robotData) {
	CoM_ = new CoMBody(generalData_, robotData_, interpolation);
	leftFoot_ = new FootBody(generalData_, robotData_, interpolation, LEFT);
	rightFoot_ = new FootBody(generalData_, robotData_, interpolation, RIGHT);
	initConvexHulls();

	currentSupport_.phase = DS;
	currentSupport_.foot = LEFT;
	currentSupport_.timeLimit = 1e9;
	currentSupport_.nbStepsLeft = 1;
	currentSupport_.stateChanged = false;
	currentSupport_.x = 0.0;
	currentSupport_.y = 0.1;
	currentSupport_.yaw = 0.0;
	currentSupport_.yawTrunk = 0.0;
	currentSupport_.startTime = 0.0;
	robotData->currentSupport = &currentSupport_;
}

RigidBodySystem::~RigidBodySystem() {
	delete rightFoot_;
	delete leftFoot_;
	delete CoM_;
}


void RigidBodySystem::computeDynamics(){
	CoM_->computeDynamics();
	leftFoot_->computeDynamics();
	rightFoot_->computeDynamics();
}

void RigidBodySystem::interpolateBodies(MPCSolution & solution, double currentTime, const VelReference & velRef){
	CoM_->interpolate(solution, currentTime, velRef);
	leftFoot_->interpolate(solution, currentTime, velRef);
	rightFoot_->interpolate(solution, currentTime, velRef);
}

void RigidBodySystem::updateBodyState(const MPCSolution & solution){
	int nextCurrentState = generalData_->iterationNumberFeedback(generalData_->MPCSamplingPeriod);

	BodyState leftFoot, rightFoot, CoM;

	for(int i = 0; i < 3; ++i){
		const MPCSolution::State & currentState = solution.state_vec[i];
		leftFoot.x(i) = currentState.leftFootTrajX_(nextCurrentState);
		leftFoot.y(i) = currentState.leftFootTrajY_(nextCurrentState);
		leftFoot.z(i) = currentState.leftFootTrajZ_(nextCurrentState);
		leftFoot.yaw(i) = currentState.leftFootTrajYaw_(nextCurrentState);

		rightFoot.x(i) = currentState.rightFootTrajX_(nextCurrentState);
		rightFoot.y(i) = currentState.rightFootTrajY_(nextCurrentState);
		rightFoot.z(i) = currentState.rightFootTrajZ_(nextCurrentState);
		rightFoot.yaw(i) = currentState.rightFootTrajYaw_(nextCurrentState);

		CoM.x(i) = currentState.CoMTrajX_(nextCurrentState);
		CoM.y(i) = currentState.CoMTrajY_(nextCurrentState);
		CoM.yaw(i) = currentState.trunkYaw_(nextCurrentState);
	}
	CoM.z(0) = robotData_->CoMHeight;
	CoM.z(1) = 0;
	CoM.z(2) = 0;

	CoM_->state(CoM);
	leftFoot_->state(leftFoot);
	rightFoot_->state(rightFoot);

}

void RigidBodySystem::firstIterationDuration(double firstIterationDuration){
	CoM_->firstIterationDuration(firstIterationDuration);
	leftFoot_->firstIterationDuration(firstIterationDuration);
	rightFoot_->firstIterationDuration(firstIterationDuration);
}

RigidBody * RigidBodySystem::body(BodyType type){
	switch(type){
		case COM:
			return CoM_;
		case LEFT_FOOT:
			return leftFoot_;
		default:
			return rightFoot_;
	}
}

const RigidBody * RigidBodySystem::body(BodyType type) const{
	switch(type){
		case COM:
			return CoM_;
		case LEFT_FOOT:
			return leftFoot_;
		default:
			return rightFoot_;
	}
}




ConvexHull RigidBodySystem::convexHull(HullType type, const SupportState & prwSupport, bool computeLinearSystem, bool rotateHull) const {
	ConvexHull hull;
	switch (type){
		case FootHull:
			if (prwSupport.foot==LEFT){
				hull = leftFootHull_;
			}else{
				hull = rightFootHull_;
			}
		break;
		case CoPHull:
			if (prwSupport.foot==LEFT){
				if (prwSupport.phase==SS){
					hull = CoPLeftSSHull_;
				}else{
					hull = CoPLeftDSHull_;
				}
			}else{
				if (prwSupport.phase==SS){
					hull = CoPRightSSHull_;
				}else{
					hull =  CoPRightDSHull_;
				}
			}
		break;
	}

	if (rotateHull){
		hull.rotate(prwSupport.yaw);
	}

	if (computeLinearSystem){
		hull.computeLinearSystem(prwSupport.foot);
	}

	return hull;
}


void RigidBodySystem::initConvexHulls() {

	  // Feet polygonal hulls:
	  int nbVertFeet = 5;

	  double DefaultFPosEdgesX[5] = {-0.28, -0.2, 0.0, 0.2, 0.28};
	  double DefaultFPosEdgesY[5] = {-0.2, -0.3, -0.4, -0.3, -0.2};

	  leftFootHull_.resize( nbVertFeet );
	  rightFootHull_.resize( nbVertFeet );

	  for(int i=0;i<nbVertFeet;++i){
		  leftFootHull_.x(i)=DefaultFPosEdgesX[i];
		  leftFootHull_.y(i)=DefaultFPosEdgesY[i];


		  rightFootHull_.x(i)=DefaultFPosEdgesX[i];
		  rightFootHull_.y(i)=-DefaultFPosEdgesY[i];
	  }




	  // ZMP polygonal hulls:
	  // TODO are those parameterizable?
	  int nbVertCoP = 4;

	  double DefaultCoPSSEdgesX[4] = {0.0686, 0.0686, -0.0686, -0.0686};
	  double DefaultCoPSSEdgesY[4] = {0.029, -0.029, -0.029, 0.029};

	  double DefaultCoPDSEdgesX[4] = {0.0686, 0.0686, -0.0686, -0.0686};
	  double DefaultCoPDSEdgesY[4] = {0.029, -0.229, -0.229, 0.029};

	  CoPLeftSSHull_.resize(nbVertCoP);
	  CoPRightSSHull_.resize(nbVertCoP);
	  CoPLeftDSHull_.resize(nbVertCoP);
	  CoPRightDSHull_.resize(nbVertCoP);

	  for(int i=0;i<nbVertCoP;++i){
		  CoPLeftSSHull_.x(i)=DefaultCoPSSEdgesX[i];
		  CoPLeftSSHull_.y(i)=DefaultCoPSSEdgesY[i];
		  CoPLeftDSHull_.x(i)=DefaultCoPDSEdgesX[i];
		  CoPLeftDSHull_.y(i)=DefaultCoPDSEdgesY[i];

		  CoPRightSSHull_.x(i)=DefaultCoPSSEdgesX[i];
		  CoPRightSSHull_.y(i)=-DefaultCoPSSEdgesY[i];
		  CoPRightDSHull_.x(i)=DefaultCoPDSEdgesX[i];
		  CoPRightDSHull_.y(i)=-DefaultCoPDSEdgesY[i];
	  }

}
