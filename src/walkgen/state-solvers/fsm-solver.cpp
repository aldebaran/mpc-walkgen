#include <mpc-walkgen/state-solvers/fsm-solver.h>

using namespace MPCWalkgen;

FSMSolver::FSMSolver(VelReference * velRef, const MPCData * generalData)
	:StateSolver(velRef, generalData)
{}

FSMSolver::~FSMSolver(){}

void FSMSolver::setSupportState(double time, int pi, SupportState & Support){

	Support.stateChanged = false;
	Support.nbInstants++;

	bool ReferenceGiven = false;
	if(fabs(velRef_->local.x)>EPS||fabs(velRef_->local.y)>EPS||fabs(velRef_->local.yaw)>EPS){
		ReferenceGiven = true;
	}

	// Update time limit for double support phase
	if(ReferenceGiven && Support.phase == DS && (Support.timeLimit-time+EPS) > generalData_->DSSSPeriod){
		//Support.TimeLimit = time+DSSSPeriod_-T_/10.0;
		Support.timeLimit = time+generalData_->DSSSPeriod;
		Support.nbStepsLeft = generalData_->nbStepSSDS;
	}

	//std::cout << "time+EPS+" << pi << "*T_ = " << time+EPS+pi*T_ << std::endl;
	//std::cout << "Support.TimeLimit = " << Support.TimeLimit << std::endl;

	//FSM
	if(time+EPS+pi*generalData_->QPSamplingPeriod >= Support.timeLimit){
		//SS->DS
		if(Support.phase == SS  && !ReferenceGiven && Support.nbStepsLeft == 0){
			Support.phase = DS;
			Support.timeLimit = time+pi*generalData_->QPSamplingPeriod+generalData_->DSPeriod;
			Support.stateChanged = true;
			Support.nbInstants = 0;
			//DS->SS
		}else if( ((Support.phase == DS) && ReferenceGiven) ||   ((Support.phase == DS) && (Support.nbStepsLeft > 0))){
			Support.phase = SS;
			Support.timeLimit = time+pi*generalData_->QPSamplingPeriod+generalData_->stepPeriod;
			Support.nbStepsLeft = generalData_->nbStepSSDS;
			Support.stateChanged = true;
			Support.nbInstants = 0;
			//SS->SS
		}else if((Support.phase == SS && Support.nbStepsLeft > 0) ||(Support.nbStepsLeft == 0 && ReferenceGiven)){
			if(Support.foot == LEFT){
				Support.foot = RIGHT;
			}else{
				Support.foot = LEFT;
			}
			Support.stateChanged = true;
			Support.nbInstants = 0;
			Support.timeLimit = time+pi*generalData_->QPSamplingPeriod+generalData_->stepPeriod;
			if(pi != 1){//Flying foot is not down
				++Support.stepNumber;
			}
			if (!ReferenceGiven){
				Support.nbStepsLeft = Support.nbStepsLeft-1;
			}
			if (ReferenceGiven){
				Support.nbStepsLeft = generalData_->nbStepSSDS;
			}
		}
	}
}
