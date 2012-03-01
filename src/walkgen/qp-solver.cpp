#include <mpc-walkgen/qp-solver.h>


using namespace MPCWalkgen;
using namespace Eigen;


//TODO: NB_VAR_MAX
const int QPSolver::DefaultNbVarMax_=100;
const int QPSolver::DefaultNbCtrMax_=100;

QPSolver::QPSolver(const int nbVarMax, const int nbCtrMax)
	:matrixQ_(0,0,nbVarMax,nbVarMax)
	,matrixA_(0,0,nbCtrMax,nbVarMax)
	,vectorP_(0,0,nbVarMax,1)
	,vectorBU_(0,0,nbCtrMax,1)
	,vectorBL_(0,0,nbCtrMax,1)
	,vectorXU_(0,0,nbVarMax,1)
	,vectorXL_(0,0,nbVarMax,1)
	,nbVar_(0)
	,nbCtr_(0)
	,nbVarMax_(nbVarMax)
	,nbCtrMax_(nbCtrMax)
	,varOrder_(nbVarMax)
	,ctrOrder_(nbVarMax+nbCtrMax)
{
	for(int i=0;i<nbVarMax;++i){
		varOrder_(i)=i;
	}
	for(int i=0;i<nbVarMax+nbCtrMax;++i){
		ctrOrder_(i)=i;
	}
}

QPSolver::~QPSolver(){}

QPMatrix & QPSolver::matrix(const QPMatrixType type){
	switch(type){
		case matrixA:
			return matrixA_;
		case vectorP:
			return vectorP_;
		case vectorBU:
			return vectorBU_;
		case vectorBL:
			return vectorBL_;
		case vectorXU:
			return vectorXU_;
		case vectorXL:
			return vectorXL_;
		default:
			return matrixQ_;
	}

}

void QPSolver::reset(const bool withConstantPart){
	matrixQ_.reset(withConstantPart);
	matrixA_.reset(withConstantPart);
	vectorP_.reset(withConstantPart);
	vectorBU_.reset(withConstantPart);
	vectorBL_.reset(withConstantPart);
	vectorXU_.reset(withConstantPart);
	vectorXL_.reset(withConstantPart);
}

void QPSolver::nbVar(const int nbVar){
	if (nbVar!=nbVar_){
		nbVar_=nbVar;
		resizeAll();
	}
}

void QPSolver::nbCtr(const int nbCtr){
	if (nbCtr!=nbCtr_){
		nbCtr_=nbCtr;
		resizeAll();
	}
}

void QPSolver::addNbCtr(const int addCtr){
	if (addCtr>0){
		nbCtr_+=addCtr;
		resizeAll();
	}
}

void QPSolver::varOrder(const Eigen::VectorXi & order){
	varOrder_ = order;
	matrixQ_.rowOrder(order);
	matrixQ_.colOrder(order);
	matrixA_.colOrder(order);
	vectorP_.rowOrder(order);
	vectorXU_.rowOrder(order);
	vectorXL_.rowOrder(order);
}

void QPSolver::ctrOrder(const Eigen::VectorXi & order){
	ctrOrder_=order;
	matrixA_.rowOrder(order);
	vectorBU_.rowOrder(order);
	vectorBL_.rowOrder(order);
}


bool QPSolver::resizeAll(){
	matrixQ_.resize(nbVar_,nbVar_);
	matrixA_.resize(nbCtr_,nbVar_);
	vectorP_.resize(nbVar_);
	vectorBU_.resize(nbCtr_);
	vectorBL_.resize(nbCtr_);
	vectorXU_.resize(nbVar_);
	vectorXL_.resize(nbVar_);

	bool maxSizechanged=false;
	if (nbVar_>nbVarMax_){
		nbVarMax_=nbVar_;
		maxSizechanged=true;
	}
	if (nbCtr_>nbCtrMax_){
		nbCtrMax_=nbCtr_;
		maxSizechanged=true;
	}
	return maxSizechanged;
}

void QPSolver::reorderInitialSolution(MPCSolution & result){
	assert(result.initialSolution.size() >= nbVar_);
	assert(result.initialConstraints.size() >= nbCtr_ + nbVar_);
	VectorXd initialSolution = result.initialSolution;
	VectorXi initialConstraints = result.initialConstraints;
	for(int i=0;i<nbVar_;++i){
		result.initialSolution(varOrder_(i))=initialSolution(i);
		result.initialConstraints(varOrder_(i))=initialConstraints(i);
	}
	for(int i=0;i<nbCtr_;++i){
		result.initialConstraints(ctrOrder_(i+nbVar_))=initialConstraints(i+nbVar_);
	}

}

void QPSolver::reorderSolution(MPCSolution & result){
	VectorXd solution = result.qpSolution;
	VectorXi constraints = result.constraints;

	for(int i=0;i<nbVar_;++i){
		result.qpSolution(i)=solution(varOrder_(i));
		result.constraints(i)=constraints(varOrder_(i));
	}
	for(int i=0;i<nbCtr_;++i){
		result.constraints(i+nbVar_)=constraints(ctrOrder_(i+nbVar_));
	}

	result.initialConstraints=	result.constraints;
}
