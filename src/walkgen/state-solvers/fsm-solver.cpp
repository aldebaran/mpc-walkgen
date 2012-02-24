#include <mpc-walkgen/state-solvers/fsm-solver.h>

using namespace MPCWalkgen;

FSMSolver::FSMSolver(VelReference * velRef, const MPCData * generalData)
	:StateSolver(velRef, generalData)
{}

FSMSolver::~FSMSolver(){}

void FSMSolver::setSupportState(double time, int sample, const std::vector<double> &samplingTimes_vec, SupportState & support) {

	support.stateChanged = false;
	support.nbInstants++;

	bool ReferenceGiven = false;
	if (fabs(velRef_->local.x)>EPS || fabs(velRef_->local.y)>EPS || fabs(velRef_->local.yaw)>EPS) {
		ReferenceGiven = true;
	}

	// Update time limit for double support phase
	if (ReferenceGiven && support.phase == DS && (support.timeLimit-time+EPS) > generalData_->DSSSPeriod) {
		//Support.TimeLimit = time+DSSSPeriod_-T_/10.0;
		support.timeLimit = time + generalData_->DSSSPeriod;
		support.nbStepsLeft = generalData_->nbStepSSDS;
	}

	//FSM
	if (time + EPS + samplingTimes_vec[sample] >= support.timeLimit){
		//SS->DS
		if (support.phase == SS && !ReferenceGiven && support.nbStepsLeft == 0){
			support.phase = DS;
			support.timeLimit = time + samplingTimes_vec[sample]+generalData_->DSPeriod;
			support.stateChanged = true;
			support.nbInstants = 0;
			//DS->SS
		} else if (((support.phase == DS) && ReferenceGiven) || ((support.phase == DS) && (support.nbStepsLeft > 0))){
			support.phase = SS;
			support.timeLimit = time+samplingTimes_vec[sample]+generalData_->stepPeriod;
			support.nbStepsLeft = generalData_->nbStepSSDS;
			support.stateChanged = true;
			support.nbInstants = 0;
			//SS->SS
		} else if ((support.phase == SS && support.nbStepsLeft > 0) || (support.nbStepsLeft == 0 && ReferenceGiven)){
			if (support.foot == LEFT){
				support.foot = RIGHT;
			} else {
				support.foot = LEFT;
			}
			support.stateChanged = true;
			support.nbInstants = 0;
			support.timeLimit = time+samplingTimes_vec[sample]+generalData_->stepPeriod;
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
