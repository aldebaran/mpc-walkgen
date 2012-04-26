#include "qpoases-solver.h"


#ifdef USE_QPOASES_3_0
# include <qpOASES/QProblem.hpp>
#else //
# include <QProblem.hpp>
#endif //USE_QPOASES_3_0

using namespace MPCWalkgen;
using namespace Eigen;


QPOasesSolver::QPOasesSolver(const int nbVarMin, const int nbCtrMin, const int nbVarMax, const int nbCtrMax)
	:QPSolver(nbVarMin, nbCtrMin, nbVarMax, nbCtrMax)
{}

QPOasesSolver::~QPOasesSolver(){}

void QPOasesSolver::solve(VectorXd & qpSolution,
			  VectorXi & constraints,
			  VectorXd & initialSolution,
			  VectorXi & initialConstraints,
			  bool useWarmStart){


	qp_ = new qpOASES::QProblem(nbVar_, nbCtr_);
	qp_->setPrintLevel(qpOASES::PL_NONE);

	reorderInitialSolution(initialSolution, initialConstraints);

	qpOASES::Constraints* ctrInit = new  qpOASES::Constraints(nbCtr_);
	qpOASES::Bounds* boundsInit = new  qpOASES::Bounds(nbVar_);
	if (useWarmStart){
		qpSolution = initialSolution;
		constraints = initialConstraints;
		for(int i=0;i<nbVar_;++i){
		      if (constraints(i)==0){
			    boundsInit->setupBound(i,qpOASES::ST_INACTIVE);
		      }else if (constraints(i)==1){
			   boundsInit->setupBound(i,qpOASES::ST_LOWER);
		      }else{
			   boundsInit->setupBound(i,qpOASES::ST_UPPER);
		      }
		}
		for(int i=0;i<nbCtr_;++i){
		      if (constraints(nbVar_+i)==0){
			   ctrInit->setupConstraint(i,qpOASES::ST_INACTIVE);
		      }else if (constraints(nbVar_+i)==1){
			   ctrInit->setupConstraint(i,qpOASES::ST_LOWER);
		      }else{
			   ctrInit->setupConstraint(i,qpOASES::ST_UPPER);
		      }
		}

	}else{
		ctrInit->setupAllInactive();
		boundsInit->setupAllFree();
		if (qpSolution.rows()!=nbVar_){
			qpSolution.setZero(nbVar_);
		}else{
			qpSolution.fill(0);
		}
		if (constraints.rows()!=nbVar_ + nbCtr_){
			constraints.setZero(nbVar_ + nbCtr_);
		}else{
			constraints.fill(0);
		}
	}

	int ittMax=100;

	MatrixXd A = matrixA_().transpose();

	qp_->init(matrixQ_().data(), vectorP_().data(), A.data(),
		  vectorXL_().data(), vectorXU_().data(),
		  vectorBL_().data(), vectorBU_().data(),
		  ittMax, 0,
		  qpSolution.data(),NULL,
		  boundsInit, ctrInit);

        double* sol = new double[nbVar_];
        qp_->getPrimalSolution(sol);
        for(int i=0;i<nbVar_;++i){
            qpSolution(i) = sol[i];
        }

        qpOASES::Constraints ctr;
        qpOASES::Bounds bounds;
        qp_->getConstraints(ctr);
        qp_->getBounds(bounds);
        for(int i=0;i<nbVar_;++i){
            if (bounds.getStatus(i)==qpOASES::ST_LOWER){
                constraints(i)=1;
            }else if (bounds.getStatus(i)==qpOASES::ST_UPPER){
                constraints(i)=2;
            }else{
                constraints(i)=0;
            }
        }
        for(int i=0;i<nbCtr_;++i){
            if (ctr.getStatus(i)==qpOASES::ST_LOWER){
                constraints(i+nbVar_)=1;
            }else if (ctr.getStatus(i)==qpOASES::ST_UPPER){
                constraints(i+nbVar_)=2;
            }else{
                constraints(i+nbVar_)=0;
            }
        }

        reorderSolution(qpSolution, constraints, initialConstraints);

	delete qp_;
	delete ctrInit;
	delete boundsInit;
}


bool QPOasesSolver::resizeAll(){
	bool maxSizechanged = QPSolver::resizeAll();

	return maxSizechanged;
}

