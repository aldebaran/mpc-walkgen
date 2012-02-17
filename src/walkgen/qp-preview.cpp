#include <mpc-walkgen/qp-preview.h>
#include <mpc-walkgen/state-solvers/fsm-solver.h>
#include <cmath>

using namespace MPCWalkgen;
using namespace Eigen;

const double QPPreview::EPS_=1e-5;


QPPreview::QPPreview(VelReference * velRef, RigidBodySystem * robot, const MPCData * generalData)
	:robot_(robot)
	,generalData_(generalData)
	,selectionMatrices_(*generalData)
	,rotationMatrix_(2*generalData_->QPNbSamplings, 2*generalData_->QPNbSamplings)
	,rotationMatrix2_(2*generalData_->QPNbSamplings, 2*generalData_->QPNbSamplings)
{
	statesolver_ = new FSMSolver(velRef, generalData);
}

QPPreview::~QPPreview(){}

void QPPreview::previewSupportStates(const double currentTime,
		const double FirstIterationDynamicsDuration, MPCSolution & result, SupportState & CurrentSupport){

	const BodyState * Foot;

	statesolver_->setSupportState( currentTime, 0, CurrentSupport);
	CurrentSupport.inTransitionPhase = false;
	if( CurrentSupport.stateChanged){
		if( CurrentSupport.foot == LEFT ){
			Foot = &robot_->body(LEFT_FOOT)->state();
		}else{
			Foot = &robot_->body(RIGHT_FOOT)->state();
		}
		CurrentSupport.x = Foot->x(0);
		CurrentSupport.y = Foot->y(0);
		CurrentSupport.yaw = Foot->yaw(0);
		CurrentSupport.startTime = currentTime;
	}
	result.supportState_vec.push_back( CurrentSupport );


	// PREVIEW SUPPORT STATES:
	// -----------------------
	// initialize the previewed support state before previewing
	SupportState PreviewedSupport = CurrentSupport;

	PreviewedSupport.stepNumber  = 0;
	for( int pi=1; pi<=generalData_->QPNbSamplings; pi++ ){
		statesolver_->setSupportState( currentTime, pi, PreviewedSupport);
		PreviewedSupport.inTransitionPhase=false;
		if( PreviewedSupport.stateChanged ){
			if( pi == 1 ){// foot down
				if( PreviewedSupport.foot == LEFT ){
					Foot = &robot_->body(LEFT_FOOT)->state();
				}else{
					Foot = &robot_->body(RIGHT_FOOT)->state();
				}
				PreviewedSupport.x = Foot->x(0);
				PreviewedSupport.y = Foot->y(0);
				PreviewedSupport.yaw = Foot->yaw(0);
				PreviewedSupport.startTime = currentTime+pi*generalData_->QPSamplingPeriod;
				if( CurrentSupport.phase == SS && PreviewedSupport.phase == SS ){
					PreviewedSupport.inTransitionPhase=true;
				}
			}
			if( /*pi > 1 &&*/ PreviewedSupport.stepNumber > 0 ){
				PreviewedSupport.x = 0.0;
				PreviewedSupport.y = 0.0;
			}
		}
		if (pi==1){
			PreviewedSupport.iterationDuration = FirstIterationDynamicsDuration;
			PreviewedSupport.iterationWeight = FirstIterationDynamicsDuration/generalData_->QPSamplingPeriod;
		}else{
			PreviewedSupport.iterationDuration = generalData_->QPSamplingPeriod;
			PreviewedSupport.iterationWeight = 1;
		}

		result.supportState_vec.push_back( PreviewedSupport );
	}


	buildSelectionMatrices(result);
}

void QPPreview::computeRotationMatrix(MPCSolution & result){
	int N = generalData_->QPNbSamplings;
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
		selectionMatrices_.V.resize(generalData_->QPNbSamplings,NbPrwSteps);
		selectionMatrices_.VT.resize(NbPrwSteps,generalData_->QPNbSamplings);
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
	for(int i=0;i<generalData_->QPNbSamplings;i++){
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
