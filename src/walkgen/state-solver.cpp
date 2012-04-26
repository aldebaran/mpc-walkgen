#include "state-solver.h"

using namespace MPCWalkgen;

StateSolver::StateSolver(VelReference * velRef, const MPCData * generalData)
	:velRef_(velRef)
	,generalData_(generalData)
{}

StateSolver::~StateSolver(){}


void StateSolver::setSupportState(double time, int sample, const std::vector<double> &samplingTimes_vec, SupportState & support) {

	support.stateChanged = false;
	support.nbInstants++;

	bool ReferenceGiven = false;
	if (fabs(velRef_->local.x(0))>EPSILON || fabs(velRef_->local.y(0))>EPSILON || fabs(velRef_->local.yaw(0))>EPSILON) {
		ReferenceGiven = true;
	}

	// Update time limit for double support phase
	if (ReferenceGiven && support.phase == DS && (support.timeLimit-time+EPSILON) > generalData_->DSSSPeriod) {
		//Support.TimeLimit = time+DSSSPeriod_-T_/10.0;
		support.timeLimit = time + generalData_->DSSSPeriod;
		support.nbStepsLeft = generalData_->nbStepSSDS;
	}

	//FSM
	if (time + EPSILON + samplingTimes_vec[sample] >= support.timeLimit) {
		//SS->DS
		if (support.phase == SS && !ReferenceGiven && support.nbStepsLeft == 0){
			support.phase 			= DS;
			support.timeLimit 		= time + samplingTimes_vec[sample] + generalData_->DSPeriod;
			support.stateChanged 	= true;
			support.nbInstants 		= 0;
			//DS->SS
		} else if (((support.phase == DS) && ReferenceGiven) || ((support.phase == DS) && (support.nbStepsLeft > 0))){
			support.phase = SS;
			support.timeLimit 		= time + samplingTimes_vec[sample] + generalData_->stepPeriod;
			support.nbStepsLeft 	= generalData_->nbStepSSDS;
			support.stateChanged 	= true;
			support.nbInstants 		= 0;
			//SS->SS
		} else if ((support.phase == SS && support.nbStepsLeft > 0) || (support.nbStepsLeft == 0 && ReferenceGiven)){
			if (support.foot == LEFT){
				support.foot = RIGHT;
			} else {
				support.foot = LEFT;
			}
			support.stateChanged 	= true;
			support.nbInstants 		= 0;
			support.timeLimit 		= time + samplingTimes_vec[sample] + generalData_->stepPeriod;
			if (sample != 1) {//Flying foot is not down
				++support.stepNumber;
			}
			if (!ReferenceGiven) {
				support.nbStepsLeft = support.nbStepsLeft-1;
			}
			if (ReferenceGiven) {
				support.nbStepsLeft = generalData_->nbStepSSDS;
			}
		}
	}
}
