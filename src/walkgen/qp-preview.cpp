#include <mpc-walkgen/qp-preview.h>
#include <mpc-walkgen/state-solvers/fsm-solver.h>
#include <cmath>

using namespace MPCWalkgen;
using namespace Eigen;

const double QPPreview::EPS_=1e-5;

//TODO:change name QPPreview to Preview
QPPreview::QPPreview(VelReference * velRef, RigidBodySystem * robot, const MPCData * generalData)
	:robot_(robot)
	,generalData_(generalData)
	,selectionMatrices_(*generalData)
	,rotationMatrix_(2*generalData_->nbSamplesQP, 2*generalData_->nbSamplesQP)
	,rotationMatrix2_(2*generalData_->nbSamplesQP, 2*generalData_->nbSamplesQP)
{
	statesolver_ = new FSMSolver(velRef, generalData);
}

QPPreview::~QPPreview()
{
	delete statesolver_;
}

void QPPreview::previewSupportStates(const double currentTime,
		const double FirstIterationDynamicsDuration, MPCSolution & result){

	const BodyState * foot;
	SupportState & currentSupport = robot_->currentSupport();

	statesolver_->setSupportState( currentTime, 0, currentSupport);
	currentSupport.inTransitionPhase = false;
	if( currentSupport.stateChanged){
		if( currentSupport.foot == LEFT ){
			foot = &robot_->body(LEFT_FOOT)->state();
		}else{
			foot = &robot_->body(RIGHT_FOOT)->state();
		}
		currentSupport.x = foot->x(0);
		currentSupport.y = foot->y(0);
		currentSupport.yaw = foot->yaw(0);
		currentSupport.startTime = currentTime;
	}
	result.supportState_vec.push_back( currentSupport );


	// PREVIEW SUPPORT STATES:
	// -----------------------
	// initialize the previewed support state before previewing
	SupportState previewedSupport = currentSupport;

	previewedSupport.stepNumber = 0;
	for( int pi=1; pi<=generalData_->nbSamplesQP; pi++ ){
		statesolver_->setSupportState( currentTime, pi, previewedSupport);
		previewedSupport.inTransitionPhase=false;
		if( previewedSupport.stateChanged ){
			if( pi == 1 ){// foot down
				if( previewedSupport.foot == LEFT ){
					foot = &robot_->body(LEFT_FOOT)->state();
				}else{
					foot = &robot_->body(RIGHT_FOOT)->state();
				}
				previewedSupport.x = foot->x(0);
				previewedSupport.y = foot->y(0);
				previewedSupport.yaw = foot->yaw(0);
				previewedSupport.startTime = currentTime+pi*generalData_->QPSamplingPeriod;
				if( currentSupport.phase == SS && previewedSupport.phase == SS ){
					previewedSupport.inTransitionPhase=true;
				}
			}
			if( /*pi > 1 &&*/ previewedSupport.stepNumber > 0 ){
				previewedSupport.x = 0.0;
				previewedSupport.y = 0.0;
			}
		}
		if (pi == 1){
			previewedSupport.iterationDuration = FirstIterationDynamicsDuration;
			previewedSupport.iterationWeight = FirstIterationDynamicsDuration/generalData_->QPSamplingPeriod;
		}else{
			previewedSupport.iterationDuration = generalData_->QPSamplingPeriod;
			previewedSupport.iterationWeight = 1;
		}

		result.supportState_vec.push_back( previewedSupport );
	}

	buildSelectionMatrices(result);
}

void QPPreview::computeRotationMatrix(MPCSolution & result){
	int N = generalData_->nbSamplesQP;
	rotationMatrix_.fill(0);
	rotationMatrix2_.fill(0);

	for( int i=0; i<N; ++i ){
		double cosYaw = cos(result.supportState_vec[i+1].yaw);
		double sinYaw = sin(result.supportState_vec[i+1].yaw);
		rotationMatrix_(i  ,i  ) =  cosYaw;
		rotationMatrix_(i+N,i  ) = -sinYaw;
		rotationMatrix_(i  ,i+N) =  sinYaw;
		rotationMatrix_(i+N,i+N) =  cosYaw;

		rotationMatrix2_(2*i  ,2*i  ) =  cosYaw;
		rotationMatrix2_(2*i+1,2*i  ) = -sinYaw;
		rotationMatrix2_(2*i  ,2*i+1) =  sinYaw;
		rotationMatrix2_(2*i+1,2*i+1) =  cosYaw;
	}
}

void QPPreview::buildSelectionMatrices(MPCSolution & result){
	const int & NbPrwSteps = result.supportState_vec.back().stepNumber;

	if (selectionMatrices_.V.cols()!=NbPrwSteps){
		selectionMatrices_.V.resize(generalData_->nbSamplesQP,NbPrwSteps);
		selectionMatrices_.VT.resize(NbPrwSteps,generalData_->nbSamplesQP);
		selectionMatrices_.VcfX.resize(NbPrwSteps);
		selectionMatrices_.VcfY.resize(NbPrwSteps);
		selectionMatrices_.Vf.resize(NbPrwSteps,NbPrwSteps);
	}

	selectionMatrices_.VcX.fill(0);
	selectionMatrices_.VcY.fill(0);
	selectionMatrices_.V.fill(0);
	selectionMatrices_.VT.fill(0);
	selectionMatrices_.VcfX.fill(0);
	selectionMatrices_.VcfY.fill(0);
	selectionMatrices_.Vf.fill(0);



	std::vector<SupportState>::iterator SS_it;
	SS_it = result.supportState_vec.begin();//points at the cur. sup. st.
	++SS_it;
	for(int i=0;i<generalData_->nbSamplesQP;i++){
		if(SS_it->stepNumber>0){
			selectionMatrices_.V(i,SS_it->stepNumber-1) = selectionMatrices_.VT(SS_it->stepNumber-1,i) = 1.0;
			if( SS_it->stepNumber==1 && SS_it->stateChanged && SS_it->phase == SS ){
				--SS_it;
				selectionMatrices_.VcfX(0) = SS_it->x;
				selectionMatrices_.VcfY(0) = SS_it->y;
				++SS_it;

				selectionMatrices_.Vf(0,0) = 1.0;
			}else if(SS_it->stepNumber>1){
				selectionMatrices_.Vf(SS_it->stepNumber-1,SS_it->stepNumber-2) = -1.0;
				selectionMatrices_.Vf(SS_it->stepNumber-1,SS_it->stepNumber-1) = 1.0;
			}
		}else{
			selectionMatrices_.VcX(i) = SS_it->x;
			selectionMatrices_.VcY(i) = SS_it->y;
		}
		++SS_it;
	}

}
