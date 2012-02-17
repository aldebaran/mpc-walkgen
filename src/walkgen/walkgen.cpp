#include <mpc-walkgen/walkgen.h>

#include <mpc-walkgen/OrientationsPreview.h>

#include <mpc-walkgen/qp-solvers/lssol-solver.h>
#include <mpc-walkgen/qp-generator.h>
#include <mpc-walkgen/qp-preview.h>
#include <mpc-walkgen/rigid-body-system.h>
#include <mpc-walkgen/interpolation.h>

#include <mpc-walkgen/mpc-debug.h>

#include <iostream>
#include <Eigen/Dense>

using namespace MPCWalkgen;
using namespace Eigen;


MPCWalkgen::WalkgenAbstract * MPCWalkgen::mpcFactory(
	const FootData & leftFoot, const FootData & rightFoot,
	const HipYawData & leftHipYaw, const HipYawData & rightHipYaw,
	double robotMass, double comHeight,
	const std::string & qpParams)
{
	MPCWalkgen::WalkgenAbstract* zmpVra =
		new Walkgen(
			leftFoot, rightFoot,
			leftHipYaw, rightHipYaw, robotMass, comHeight,
			qpParams);
	return zmpVra;
}

using namespace MPCWalkgen;

WalkgenAbstract::WalkgenAbstract(
	  const FootData & // leftFoot
	, const FootData & // rightFoot
	, const HipYawData & // leftHipYaw
	, const HipYawData & // rightHipYaw
	, double // robotMass
	, double // comHeight
) {}

WalkgenAbstract::~WalkgenAbstract(){}


// Implementation of the private interface
Walkgen::Walkgen(
		const FootData & leftFoot, const FootData & rightFoot,
		const HipYawData & leftHipYaw, const HipYawData & rightHipYaw,
		double robotMass, double comHeight,
		const std::string & qpParams)
	: WalkgenAbstract(leftFoot, rightFoot,
		leftHipYaw, rightHipYaw, robotMass, comHeight)
	, generalData_()
	,robotData_()
	,solver_(0x0)
	,generator_(0x0)
	,preview_(0x0)
	,interpolation_(0x0)
	,robot_(0x0)
	,orientPrw_(0x0)
	,solution_()
	,velRef_()
	,ponderation_(2)
	,newCurrentSupport_()
	,isNewCurrentSupport_(false)
	,debug_(0x0)
	,enableDisplay_(true)
	,upperTimeLimitToUpdate_(0)
	,upperTimeLimitToFeedback_(0)
{
	generalData_.QPNbSamplings = 16;
	generalData_.QPSamplingPeriod = 0.1;
	generalData_.MPCSamplingPeriod = 0.005;
	generalData_.simSamplingPeriod = 0.005;
	generalData_.stepPeriod = 0.8;
	generalData_.nbStepSSDS = 2;
	generalData_.DSPeriod = 1e9;
	generalData_.DSSSPeriod = 0.8;

	if(qpParams != "")
	  parseQPParams(qpParams);

	robotData_.CoMHeight = comHeight;
	robotData_.freeFlyingFootMaxHeight = 0.05;
	robotData_.leftFoot = leftFoot;
	robotData_.rightFoot = rightFoot;
	robotData_.leftHipYaw = leftHipYaw;
	robotData_.rightHipYaw = rightHipYaw;
	robotData_.robotMass = robotMass;


	
	
	solver_ = new LSSOLSolver();

	orientPrw_ = new OrientationsPreview( robotData_.leftHipYaw, robotData_.rightHipYaw );

	interpolation_ = new Interpolation();

	robot_ = new RigidBodySystem(&generalData_, &robotData_, interpolation_);

	preview_ = new QPPreview(&velRef_,robot_, &generalData_ );

	generator_= new QPGenerator(preview_, solver_, &velRef_, &ponderation_, robot_, &generalData_ );

	debug_ = new MPCDebug(true);
}


Walkgen::~Walkgen(){
	if (debug_!=0x0)
		delete debug_;

	if (orientPrw_!=0x0)
		delete orientPrw_;

	if (solver_!=0x0)
		delete solver_;

	if (generator_!=0x0)
		delete generator_;

	if (preview_!=0x0)
		delete preview_;

	if (robot_!=0x0)
		delete robot_;

	if (interpolation_!=0x0)
		delete interpolation_;

}

void Walkgen::init(const Eigen::Vector3d & leftFootPosition, const Eigen::Vector3d & rightFootPosition){


	//Check if sampling periods are defined correctly
	assert(generalData_.simSamplingPeriod > 0);
	assert(generalData_.MPCSamplingPeriod >= generalData_.simSamplingPeriod);
	assert(generalData_.QPSamplingPeriod >= generalData_.MPCSamplingPeriod);

	VectorXi order(QPSolver::DefaultNbVarMax_);
	for(int i=0;i<generalData_.QPNbSamplings;++i){
		order(i)=2*i;
		order(i+generalData_.QPNbSamplings)=2*i+1;
	}
	for(int i=2*generalData_.QPNbSamplings;i<QPSolver::DefaultNbVarMax_;++i){
		order(i)=i;
	}
	solver_->varOrder(order);

	ponderation_.CopCentering[0]   =0.0001;
	ponderation_.JerkMin[0]        =0.001;
	ponderation_.instantVelocity[0]=1;

	ponderation_.CopCentering[1]   =10;
	ponderation_.JerkMin[1]        =0.001;
	ponderation_.instantVelocity[1]=1;

	ponderation_.activePonderation = 1;

	orientPrw_->SamplingPeriod( generalData_.QPSamplingPeriod );
	orientPrw_-> SimuPeriod(generalData_.MPCSamplingPeriod);
	orientPrw_->NbSamplingsPreviewed( generalData_.QPNbSamplings );
	orientPrw_->SSLength( generalData_.stepPeriod );

	robot_->computeDynamics();

	generator_->precomputeObjective();

	solution_.currentSupportState.phase = DS;
	solution_.currentSupportState.foot = LEFT;
	solution_.currentSupportState.timeLimit = 1e9;
	solution_.currentSupportState.nbStepsLeft = 1;
	solution_.currentSupportState.stateChanged = false;
	solution_.currentSupportState.x = 0.0;//TODO: hard coded!
	solution_.currentSupportState.y = 0.1;
	solution_.currentSupportState.yaw = 0.0;
	solution_.currentSupportState.yawTrunk = 0.0;

	solution_.currentSupportState.startTime = 0.0;

	BodyState state;
	state.x[0]=leftFootPosition[0];
	state.y[0]=leftFootPosition[1];
	robot_->body(LEFT_FOOT)->state(state);

	state.x[0]=rightFootPosition[0];
	state.y[0]=rightFootPosition[1];
	robot_->body(RIGHT_FOOT)->state(state);

	state.x[0]=0;
	state.y[0]=0;
	state.z[0]=robotData_.CoMHeight;
	robot_->body(COM)->state(state);

	upperTimeLimitToUpdate_ = 0.0;
	upperTimeLimitToFeedback_ = 0.0;
}

const MPCSolution & Walkgen::online(double time, bool previewBodiesNextState){
	solution_.newTraj = false;
	if(time  > upperTimeLimitToUpdate_+EPS){
		upperTimeLimitToUpdate_ += generalData_.QPSamplingPeriod;
		CurrentQPTime_ = time;
	}

	if(time  > upperTimeLimitToFeedback_+EPS){

		debug_->getTime(1,true);

		solver_->reset();

		solution_.reset();
		solution_.newTraj = true;

		velRef_ = newVelRef_;
		if (isNewCurrentSupport_){
			solution_.currentSupportState = newCurrentSupport_;
			isNewCurrentSupport_=false;
		}

		if (solution_.currentSupportState.phase==SS && solution_.currentSupportState.nbStepsLeft == 0){
			velRef_.local.x=0;
			velRef_.local.y=0;
			velRef_.local.yaw=0;
		}
		if (velRef_.local.yaw==0 && velRef_.local.x==0 && velRef_.local.y==0){
			ponderation_.activePonderation = 1;
		}else{
			ponderation_.activePonderation = 0;
		}

		double FirstIterationDynamicsDuration = upperTimeLimitToUpdate_-upperTimeLimitToFeedback_;
		upperTimeLimitToFeedback_ += generalData_.MPCSamplingPeriod;

		robot_->firstIterationDuration(FirstIterationDynamicsDuration);

		preview_->previewSupportStates(CurrentQPTime_, FirstIterationDynamicsDuration, solution_, solution_.currentSupportState);

		orientPrw_->preview_orientations( CurrentQPTime_, velRef_,
			generalData_.stepPeriod,
			robot_->body(LEFT_FOOT)->state(), robot_->body(RIGHT_FOOT)->state(),
			solution_ );

		preview_->computeRotationMatrix(solution_);


		generator_->computeReferenceVector(solution_);
		generator_->buildObjective(solution_);
		generator_->buildConstraints(solution_);
		generator_->computeWarmStart(solution_);

		debug_->getTime(1,false);
		debug_->getTime(2,true);

		solver_->solve(solution_);

		debug_->getTime(2,false);
		debug_->getTime(3,true);

		if (enableDisplay_)
			generator_->display(solution_, "pg-data-displayer.dat");

		debug_->getTime(3,false);

		debug_->getTime(4,true);
		generator_->convertCopToJerk(solution_);


		robot_->interpolateBodies(solution_, time, velRef_);

		if (previewBodiesNextState){
			robot_->updateBodyState(solution_);
		}


		// Only for compatibility with temporary class OrientationPreview
		orientPrw_->interpolate_trunk_orientation(robot_);

		debug_->getTime(4,false);


		if (debug_->nbIntervals(1)==100){
			std::cout << "prepa    :" << debug_->computeInterval(1,us) << " us" << std::endl;
			std::cout << "solve    :" << debug_->computeInterval(2,us) << " us" << std::endl;
			//std::cout << "print    :" << debug_->computeInterval(3,us) << " us" << std::endl;
			std::cout << "inter    :" << debug_->computeInterval(4,us) << " us" << std::endl<< std::endl;

			debug_->reset();
		}


	}

	return solution_;
}

void Walkgen::reference(double dx, double dy, double dyaw){
	newVelRef_.local.x = dx;
	newVelRef_.local.y = dy;
	newVelRef_.local.yaw = dyaw;
}

const BodyState & Walkgen::bodyState(BodyType body)const{
	return robot_->body(body)->state();
}
void Walkgen::bodyState(BodyType body, const BodyState & state){
	robot_->body(body)->state(state);
}

void Walkgen::parseQPParams (const std::string &)
{
	std::cerr << " The method Walkgen::parseQPParams is not implemented yet. " << std::endl;
}

void Walkgen::QPSamplingPeriod(double)
{
	std::cerr << " The method Walkgen::QPSamplingPeriod(d) is not implemented yet. " << std::endl;
}

void Walkgen::mpcSamplingPeriod(double)
{
	std::cerr << " The method Walkgen::mpcSamplingPeriod(d) is not implemented yet. " << std::endl;
}

void Walkgen::simSamplingPeriod(double)
{
	std::cerr << " The method Walkgen::simSamplingPeriod(d) is not implemented yet. " << std::endl;
}

void Walkgen::QPNbSamplings(int)
{
	std::cerr << " The method Walkgen::QPNbSamplings(d) is not implemented yet. " << std::endl;
}
