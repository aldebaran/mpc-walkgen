	#include <mpc-walkgen/qp-generator.h>
#include <mpc-walkgen/qp-matrix.h>
#include <mpc-walkgen/tools.h>

#include <iostream>
#include <fstream>


using namespace MPCWalkgen;
using namespace Eigen;

QPGenerator::QPGenerator(QPPreview * preview, QPSolver * solver,
		VelReference * velRef, QPPonderation * ponderation,
		RigidBodySystem * robot, const MPCData * generalData)
	:preview_(preview)
	,solver_(solver)
	,robot_(robot)
	,velRef_(velRef)
	,ponderation_(ponderation)
	,generalData_(generalData)
	,tmpVec_(1)
	,tmpVec2_(1)
	,tmpMat_(1,1)
	,tmpMat2_(1,1)
{
}

QPGenerator::~QPGenerator(){}


void QPGenerator::precomputeObjective(){

	int N = generalData_->QPNbSamplings;
	int pondSize = ponderation_->JerkMin.size();
	int size=generalData_->nbIterationFeedback();
	Qconst_.resize(size*pondSize);
	choleskyConst_.resize(size*pondSize);
	pconstCoM_.resize(size*pondSize);
	pconstVc_.resize(size*pondSize);
	pconstRef_.resize(size*pondSize);

	MatrixXd pondFactor = MatrixXd::Identity(N,N);
	MatrixXd G(N,N);

	VectorXi order(2*N);
	for(int i=0;i<N;++i){
		order(i)=2*i;
		order(i+N)=2*i+1;
	}

	QPMatrix chol(2*N,2*N,2*N,2*N);
	chol.rowOrder(order);
	chol.colOrder(order);

	for(int i=0; i<pondSize; ++i){
		for(double s=generalData_->MPCSamplingPeriod;
				s<generalData_->QPSamplingPeriod+EPS;
				s+=generalData_->MPCSamplingPeriod){
			int nb = (int)round(s / generalData_->MPCSamplingPeriod)-1;
			nb+=i*size;
			robot_->firstIterationDuration(s);

			const DynamicMatrix & CoPDynamics = robot_->body(COM)->dynamic(copDynamic);
			const DynamicMatrix & VelDynamics = robot_->body(COM)->dynamic(velDynamic);

			double firstIterationWeight = s/generalData_->QPSamplingPeriod;
			pondFactor(0,0)=firstIterationWeight;
			pondFactor(N-1,N-1)=1.05-firstIterationWeight;


			tmpMat_ = CoPDynamics.UInvT*VelDynamics.UT*pondFactor*VelDynamics.U*CoPDynamics.UInv;
			tmpMat2_= CoPDynamics.UInvT*pondFactor*CoPDynamics.UInv;

			G = ponderation_->instantVelocity[i]*tmpMat_ + ponderation_->JerkMin[i]*tmpMat2_;
			Qconst_[nb]=G;

			tmpMat_ = G + ponderation_->CopCentering[i]*MatrixXd::Identity(N,N);

			chol.reset();
			chol.addTerm(tmpMat_,0,0);
			chol.addTerm(tmpMat_,N,N);
			choleskyConst_[nb]=chol.cholesky();

			pconstCoM_[nb] = VelDynamics.S - VelDynamics.U*CoPDynamics.UInv*CoPDynamics.S;
			pconstCoM_[nb] = CoPDynamics.UInvT*VelDynamics.UT*ponderation_->instantVelocity[i]*pondFactor*pconstCoM_[nb];
			pconstCoM_[nb]-= CoPDynamics.UInvT*ponderation_->JerkMin[i]*pondFactor*CoPDynamics.UInv*CoPDynamics.S;

			pconstVc_[nb]  = CoPDynamics.UInvT*ponderation_->JerkMin[i]*pondFactor*CoPDynamics.UInv;
			pconstVc_[nb] += CoPDynamics.UInvT*VelDynamics.UT*ponderation_->instantVelocity[i]*pondFactor*VelDynamics.U*CoPDynamics.UInv;

			pconstRef_[nb] = -CoPDynamics.UInvT*VelDynamics.UT*ponderation_->instantVelocity[i]*pondFactor;


		}
	}
}

void QPGenerator::buildObjective(const MPCSolution & result){


	int nb = generalData_->iterationNumberFeedback(result.supportState_vec[1].iterationDuration);
	nb += ponderation_->activePonderation*generalData_->nbIterationFeedback();
	const BodyState & CoM = robot_->body(COM)->state();
	const SelectionMatrices & state = preview_->selectionMatrices();
	const MatrixXd & rot = preview_->rotationMatrix();
	const MatrixXd & rot2 = preview_->rotationMatrix2();

	int nbStepsPreviewed = result.supportState_vec.back().stepNumber;
	int N = generalData_->QPNbSamplings;

	solver_->nbVar(2*N+2*nbStepsPreviewed);
	solver_->nbCtr(0);

	if (nbStepsPreviewed>0){
		tmpMat_ = Qconst_[nb]*state.V;
		solver_->matrix(matrixQ).addTerm(tmpMat_,0,2*N);
		solver_->matrix(matrixQ).addTerm(tmpMat_,N,2*N+nbStepsPreviewed);


		tmpMat_ = state.VT*Qconst_[nb]*state.V;
		solver_->matrix(matrixQ).addTerm(tmpMat_, 2*N , 2*N );
		solver_->matrix(matrixQ).addTerm(tmpMat_, 2*N+nbStepsPreviewed, 2*N+nbStepsPreviewed );

		MatrixXd & Q = solver_->matrix(matrixQ)();

		// rotate the down left block
		MatrixXd dlBlock = Q.block(2*N,0,2*nbStepsPreviewed,2*N);
		computeMRt(dlBlock, rot2);
		Q.block(2*N,0,2*nbStepsPreviewed,2*N) = dlBlock;

		// rotate the upper right block
		MatrixXd urBlock = Q.block(0,2*N,2*N,2*nbStepsPreviewed);
		computeRM (urBlock, rot2);
		Q.block(0,2*N,2*N,2*nbStepsPreviewed) = urBlock;
	}

	// rotate the cholesky matrix
	MatrixXd chol = choleskyConst_[nb];
	rotateCholeskyMatrix(chol, rot2);
	solver_->matrix(matrixQ).cholesky(chol);


	VectorXd HX(N),HY(N),H(2*N);

	HX = pconstCoM_[nb]*CoM.x;
	HY = pconstCoM_[nb]*CoM.y;

	HX += pconstVc_[nb]*state.VcX;
	HY += pconstVc_[nb]*state.VcY;

	HX += pconstRef_[nb]*velRef_->global.xVec;
	HY += pconstRef_[nb]*velRef_->global.yVec;

	if (nbStepsPreviewed>0){
		tmpVec_ = state.VT*HX;
		solver_->matrix(vectorP).addTerm(tmpVec_, 2*N);
		tmpVec_ = state.VT*HY;
		solver_->matrix(vectorP).addTerm(tmpVec_, 2*N+nbStepsPreviewed);
	}


	H << HX, HY;
	H = rot*H;

	solver_->matrix(vectorP).addTerm(H, 0 );

}

void QPGenerator::buildConstraints(const MPCSolution & result){
	int nbStepsPreviewed = result.supportState_vec.back().stepNumber;

	buildConstraintsCOP(result);
	if (nbStepsPreviewed>0){
		buildInequalitiesFeet(result);
		buildConstraintsFeet(result);
	}
}

void QPGenerator::computeWarmStart(MPCSolution & result){

	// Initialize:
	// -----------
	int nbSteps = result.supportState_vec.back().stepNumber;
	int nbStepsMax = 4;
	int nbSampling = generalData_->QPNbSamplings;
	result.initialSolution.resize(4*nbSampling+4*nbSteps);

	// Compute previewed initial constraints:
	// ---------------------
	int size=result.initialConstraints.rows();
	VectorXi initialConstraintTmp = result.initialConstraints;
	double TimeFactor = result.supportState_vec[1].iterationWeight;
	int shift_ctr;
	if (fabs(TimeFactor-1.)<EPS){
		shift_ctr=1;
	}else{
		shift_ctr=0;
	}

	if (size>=2*nbSampling){
		result.initialConstraints.segment(0,          nbSampling-1) = initialConstraintTmp.segment(shift_ctr,    nbSampling-1);
		result.initialConstraints.segment(nbSampling, nbSampling-1) = initialConstraintTmp.segment(shift_ctr+nbSampling, nbSampling-1);

		result.initialConstraints(  nbSampling-1)=initialConstraintTmp(  nbSampling-1);
		result.initialConstraints(2*nbSampling-1)=initialConstraintTmp(2*nbSampling-1);

		result.initialConstraints.segment(2*nbSampling          , 5*nbSteps)=
			initialConstraintTmp.segment (2*nbSampling          , 5*nbSteps);
		result.initialConstraints.segment(2*nbSampling+5*nbSteps, 5*(nbStepsMax-nbSteps))=
			initialConstraintTmp.segment (2*nbSampling+5*nbSteps, 5*(nbStepsMax-nbSteps));
	}else{
		result.initialConstraints = VectorXi::Zero(2*nbSampling+5*nbStepsMax);
	}


	// Compute feasible initial ZMP and foot positions:
	// ---------------------------------------
	std::vector<SupportState>::iterator prwSS_it = result.supportState_vec.begin();
	prwSS_it++;//Point at the first previewed support state
	// Copy current support
	SupportState currentSupport = result.supportState_vec.front();
	// if in transition phase
	if (prwSS_it->stateChanged){
	currentSupport=*prwSS_it;
	}
	int j = 0;
	ConvexHull FootFeasibilityEdges, COPFeasibilityEdges;
	double shiftx,shifty;
	bool noActiveConstraints;
	for(int i=0; i<nbSampling; i++){
		// Get COP convex hull for current support
		COPFeasibilityEdges = robot_->convexHull(CoPHull,*prwSS_it, false);
		// Check if the support foot has changed
		if (prwSS_it->stateChanged && prwSS_it->stepNumber>0){

			// Get feet convex hull for current support
			prwSS_it--;
			FootFeasibilityEdges = robot_->convexHull(FootHull,*prwSS_it, false);
			prwSS_it++;



			// Place the foot on active constraints
			shiftx=shifty=0;
			noActiveConstraints=true;
			for(int k=0;k<5;++k){
				if (result.initialConstraints(k+2*nbSampling+j*5)!=0){
					int k2=(k+1)%5; // k(4) = k(0)
					if (result.initialConstraints(k2+2*nbSampling+j*5)!=0){
						shiftx=FootFeasibilityEdges.x(k2);
						shifty=FootFeasibilityEdges.y(k2);
					}else{
						shiftx=(FootFeasibilityEdges.x(k)+FootFeasibilityEdges.x(k2))/2;
						shifty=(FootFeasibilityEdges.y(k)+FootFeasibilityEdges.y(k2))/2;
					}
					noActiveConstraints=false;
					break;
				}
			}
			if (noActiveConstraints){
				shiftx=(FootFeasibilityEdges.x(4)+FootFeasibilityEdges.x(0))/2;
				shifty=(FootFeasibilityEdges.y(4)+FootFeasibilityEdges.y(2))/2;
			}

			currentSupport.x += shiftx;
			currentSupport.y += shifty;

			// Set the new position into initial solution vector
			result.initialSolution(2*nbSampling+j) = currentSupport.x;
			result.initialSolution(2*nbSampling+nbSteps+j) = currentSupport.y;
			++j;
		}
		// Place the ZMP on active constraints
		shiftx=shifty=0;
		noActiveConstraints=true;
		int k1=-1;
		int k2=-1;
		if (result.initialConstraints(0+i*2)==1){
			if (result.initialConstraints(nbSampling+i*2)==1){
				k2=1;
				noActiveConstraints=false;
			}else if (result.initialConstraints(nbSampling+i*2)==2){
				k2=0;
				noActiveConstraints=false;
			}else if (result.initialConstraints(nbSampling+i*2)==0){
				k1=0;
				k2=1;
				noActiveConstraints=false;
			}
		}else if (result.initialConstraints(0+i*2)==2){
			if (result.initialConstraints(nbSampling+i*2)==1){
				k2=2;
				noActiveConstraints=false;
			}else if (result.initialConstraints(nbSampling+i*2)==2){
				k2=3;
				noActiveConstraints=false;
			}else if (result.initialConstraints(nbSampling+i*2)==0){
				k1=3;
				k2=2;
				noActiveConstraints=false;
			}
		}else if (result.initialConstraints(nbSampling+i*2)==1){
			k1=2;
			k2=1;
			noActiveConstraints=false;
		}else if (result.initialConstraints(nbSampling+i*2)==2){
			k1=0;
			k2=3;
			noActiveConstraints=false;
		}

		if (!noActiveConstraints){
			if (k1!=-1){
				shiftx=(COPFeasibilityEdges.x[k1]+COPFeasibilityEdges.x[k2])/2;
				shifty=(COPFeasibilityEdges.y[k1]+COPFeasibilityEdges.y[k2])/2;
			}else{
				shiftx=COPFeasibilityEdges.x[k2];
				shifty=COPFeasibilityEdges.y[k2];
			}
		}


		result.initialSolution(i) = shiftx;
		result.initialSolution(nbSampling+i) = shifty;
		++prwSS_it;

	}

}


void QPGenerator::computeReferenceVector(const MPCSolution & result){

	if (velRef_->global.xVec.rows()!=generalData_->QPNbSamplings){
		velRef_->global.xVec.resize(generalData_->QPNbSamplings);
		velRef_->global.yVec.resize(generalData_->QPNbSamplings);
	}

	  double YawTrunk;
	for (int i=0;i<generalData_->QPNbSamplings;++i){
		YawTrunk = result.supportState_vec[i+1].yaw;
		velRef_->global.xVec(i) = velRef_->local.x*cos(YawTrunk)-velRef_->local.y*sin(YawTrunk);
		velRef_->global.yVec(i) = velRef_->local.x*sin(YawTrunk)+velRef_->local.y*cos(YawTrunk);
	}

}

void QPGenerator::convertCopToJerk(MPCSolution & result){
	int N = generalData_->QPNbSamplings;

	const SelectionMatrices & State = preview_->selectionMatrices();
	const MatrixXd & rot = preview_->rotationMatrix();
	const BodyState & CoM = robot_->body(COM)->state();
	const DynamicMatrix & CoP = robot_->body(COM)->dynamic(copDynamic);
	int nbSteps =  result.supportState_vec.back().stepNumber;

	VectorXd sx = result.solution.segment(0, N);
	VectorXd sy = result.solution.segment(N, N);

	const VectorXd px = result.solution.segment(2*N,         nbSteps);
	const VectorXd py = result.solution.segment(2*N+nbSteps, nbSteps);

	VectorXd Vpx;
	VectorXd Vpy;
	if (nbSteps>0){
		Vpx =State.V*px;
		Vpy =State.V*py;
	}else{
		Vpx=VectorXd::Zero(N);
		Vpy=VectorXd::Zero(N);
	}

	VectorXd zx(N);
	zx =rot.block(0,0,N,N)*sx -rot.block(0,N,N,N)*sy;
	zx+=Vpx+State.VcX;

	VectorXd zy(N);
	zy =rot.block(N,N,N,N)*sy -rot.block(N,0,N,N)*sx;
	zy+=Vpy+State.VcY;

	VectorXd X;
	VectorXd Y;
	zx -= CoP.S * CoM.x;
	X  =  CoP.UInv * zx;

	zy -= CoP.S * CoM.y ;
	Y  =  CoP.UInv * zy;

	result.solution.segment(0, N) = X.segment(0, N);
	result.solution.segment(N, N) = Y.segment(0, N);


}

void QPGenerator::display(const MPCSolution & result, const std::string & filename) const
{
	int N = generalData_->QPNbSamplings;

	const SelectionMatrices & State = preview_->selectionMatrices();
	const MatrixXd & rot = preview_->rotationMatrix();
	const BodyState & CoM = robot_->body(COM)->state();
	const DynamicMatrix & CoP = robot_->body(COM)->dynamic(copDynamic);
	int nbSteps =  result.supportState_vec.back().stepNumber;

	VectorXd sx = result.solution.segment(0, N);
	VectorXd sy = result.solution.segment(N, N);

	const VectorXd px = result.solution.segment(2*N,         nbSteps);
	const VectorXd py = result.solution.segment(2*N+nbSteps, nbSteps);

	VectorXd Vpx;
	VectorXd Vpy;
	if (nbSteps>0){
		Vpx =State.V*px;
		Vpy =State.V*py;
	}else{
		Vpx=VectorXd::Zero(N);
		Vpy=VectorXd::Zero(N);
	}

	VectorXd zx(N);
	zx =rot.block(0,0,N,N)*sx -rot.block(0,N,N,N)*sy;
	zx+=Vpx+State.VcX;

	VectorXd zy(N);
	zy =rot.block(N,N,N,N)*sy -rot.block(N,0,N,N)*sx;
	zy+=Vpy+State.VcY;

	VectorXd X;
	VectorXd Y;
	zx -= CoP.S * CoM.x;
	X  =  CoP.UInv * zx;

	zy -= CoP.S * CoM.y ;
	Y  =  CoP.UInv * zy;



	std::ofstream data(filename.c_str());
	if (!data)
	{
		std::cerr << "Unable to open " << filename << std::endl;
		return;
	}

	int nbSampling = generalData_->QPNbSamplings;

	VectorXd ZX(nbSampling);
	VectorXd ZY(nbSampling);
	VectorXd CX(nbSampling);
	VectorXd CY(nbSampling);



	const DynamicMatrix & CoMPos = robot_->body(COM)->dynamic(posDynamic);


	// Compute previewed ZMP
	ZX=CoP.S * CoM.x + CoP.U*X;
	ZY=CoP.S * CoM.y + CoP.U*Y;

	CX = CoMPos.S*CoM.x + CoMPos.U*X;
	CY = CoMPos.S*CoM.y + CoMPos.U*Y;

	//display previewed ZMP

	for(int i=0;i<generalData_->QPNbSamplings;++i){
		data << "TRAJ\t1\t\t0\t1\t1\t\t" << ZX(i) << "\t" << ZY(i) << "\t0\n";
	}

	//display previewed COM

	for(int i=0;i<generalData_->QPNbSamplings;++i){
		data << "TRAJ\t2\t\t1\t0\t0\t\t" << CX(i) << "\t" << CY(i) << "\t0\n";
	}



	SupportState currentSupport = result.supportState_vec.front();
	std::vector<SupportState>::const_iterator prwSS_it = result.supportState_vec.begin();


	 int j = 0,b=0;


	double Xfoot, Yfoot;

	//display current COP constraint
	ConvexHull COPFeasibilityEdges = robot_->convexHull(CoPHull, *prwSS_it, false);

	for(int k=0;k<4;++k){
		  data << "BOUND\t-1\t\t0.1\t0.8\t0.1\t\t" <<
				  COPFeasibilityEdges.x[k]+currentSupport.x << "\t" <<
				  COPFeasibilityEdges.y[k]+currentSupport.y << "\t0\n";
	 }
	 data << "BOUND\t-1\t\t0.1\t0.8\t0.1\t\t" <<
	   COPFeasibilityEdges.x[0]+currentSupport.x << "\t" <<
	   COPFeasibilityEdges.y[0]+currentSupport.y << "\t0\n";

	  //display current feet positions
	  data << "POINT\t-1\t\t0.1\t0.8\t0.1\t\t" <<
			  currentSupport.x << "\t" <<
			  currentSupport.y << "\t0\n";
	  ++b;

	prwSS_it++;

	for(int i=0; i<generalData_->QPNbSamplings; i++)
	{

	  //display constraints
	  if (prwSS_it->stateChanged){

		  COPFeasibilityEdges = robot_->convexHull(CoPHull, *prwSS_it, false);
		  ConvexHull FootFeasibilityEdges = robot_->convexHull(FootHull, *prwSS_it, false);

		  if(prwSS_it->stepNumber==0){
				Xfoot=result.supportState_vec[0].x;
				Yfoot=result.supportState_vec[0].y;
		  }else{

				Xfoot=result.solution(2*generalData_->QPNbSamplings+j);
				Yfoot=result.solution(2*generalData_->QPNbSamplings+nbSteps+j);
				if (j+1<nbSteps){
					j++;
				}
		  }

		  if (prwSS_it->inTransitionPhase){
			  Xfoot=result.supportState_vec[1].x;
			  Yfoot=result.supportState_vec[1].y;
		  }



			  //display COP constraints
			  for(int k=0;k<4;++k){
				  data << "BOUND\t" << b << "\t\t0.5\t0.5\t0.5\t\t" <<
						  COPFeasibilityEdges.x[k]+Xfoot << "\t" <<
						  COPFeasibilityEdges.y[k]+Yfoot << "\t0\n";
			  }

			  //display feet constraints
			  for(int k=0;k<5;++k){
				  data << "BOUND\t" << b+10000 << "\t\t0.5\t0.5\t0.5\t\t" <<
						  FootFeasibilityEdges.x[k]+Xfoot << "\t" <<
						  FootFeasibilityEdges.y[k]+Yfoot << "\t0\n";
			  }

			  //display feet positions
			  data << "POINT\t" << b+100000 << "\t\t1\t0\t0\t\t" <<
					  Xfoot << "\t" <<
					  Yfoot << "\t0\n";
			  ++b;
	  }
	  prwSS_it++;

	}
	data.close();
}

void QPGenerator::buildInequalitiesFeet(const MPCSolution & result){

	int nbIneq = 5;
	int nbSteps = result.supportState_vec.back().stepNumber;

	feetInequalities_.resize(nbIneq*nbSteps , nbSteps);

	ConvexHull hull;

	std::vector<SupportState>::const_iterator prwSS_it = result.supportState_vec.begin();
	prwSS_it++;//Point at the first previewed instant
	for( int i=0; i<generalData_->QPNbSamplings; ++i ){
		//foot positioning constraints
		if( prwSS_it->stateChanged && prwSS_it->stepNumber>0 && prwSS_it->phase != DS){

			prwSS_it--;//Take the support state before
			hull = robot_->convexHull(FootHull, *prwSS_it);
			prwSS_it++;

			int stepNumber = (prwSS_it->stepNumber-1);

			feetInequalities_.DX.block( stepNumber*nbIneq, stepNumber, nbIneq, 1) = hull.A.segment(0, nbIneq);
			feetInequalities_.DY.block( stepNumber*nbIneq, stepNumber, nbIneq, 1) = hull.B.segment(0, nbIneq);
			feetInequalities_.Dc.segment(stepNumber*nbIneq, nbIneq) = hull.D.segment(0, nbIneq);
		}
		prwSS_it++;
	}

}

void QPGenerator::buildConstraintsFeet(const MPCSolution & result){

	int nbStepsPreviewed = result.supportState_vec.back().stepNumber;

	const SelectionMatrices & State = preview_->selectionMatrices();

	int nbCtr = solver_->nbCtr();
	solver_->addNbCtr(5*nbStepsPreviewed);

	tmpMat_.noalias() = feetInequalities_.DX*State.Vf;
	solver_->matrix(matrixA).addTerm(tmpMat_,nbCtr, 2*generalData_->QPNbSamplings);

	tmpMat_.noalias() = feetInequalities_.DY*State.Vf;
	solver_->matrix(matrixA).addTerm(tmpMat_,nbCtr, 2*generalData_->QPNbSamplings+nbStepsPreviewed);


	solver_->matrix(vectorBL).addTerm(feetInequalities_.Dc,nbCtr);

	tmpVec_ =  feetInequalities_.DX*State.VcfX;
	tmpVec_ += feetInequalities_.DY*State.VcfY;
	solver_->matrix(vectorBL).addTerm(tmpVec_,nbCtr);

	solver_->matrix(vectorBU)().block(nbCtr,0,tmpVec_.size(),1).fill(10e10);
}

void QPGenerator::buildConstraintsCOP(const MPCSolution & result){

	int nbSampling = generalData_->QPNbSamplings;
	std::vector<SupportState>::const_iterator prwSS_it = result.supportState_vec.begin();

	ConvexHull hull = robot_->convexHull(CoPHull, *prwSS_it, false, false);

	int nbStepsPreviewed = result.supportState_vec.back().stepNumber;
	int size = 2*generalData_->QPNbSamplings+2*nbStepsPreviewed;
	tmpVec_.resize(size);
	tmpVec2_.resize(size);


	++prwSS_it;//Point at the first previewed instant
	for(int i=0; i<generalData_->QPNbSamplings; ++i ){
		if( prwSS_it->stateChanged ){
			hull = robot_->convexHull(CoPHull, *prwSS_it, false, false);
		}
		tmpVec_(i)    = std::min(hull.x(0),hull.x(3));
		tmpVec2_(i)   = std::max(hull.x(0),hull.x(3));

		tmpVec_(generalData_->QPNbSamplings+i) = std::min(hull.y(0),hull.y(1));
		tmpVec2_(generalData_->QPNbSamplings+i)= std::max(hull.y(0),hull.y(1));
		++prwSS_it;
	}

	tmpVec_.segment( 2*nbSampling, 2*nbStepsPreviewed).fill(-10e10);
	tmpVec2_.segment(2*nbSampling, 2*nbStepsPreviewed).fill(10e10);

	solver_->matrix(vectorXL).addTerm(tmpVec_,0);
	solver_->matrix(vectorXU).addTerm(tmpVec2_,0);


}
