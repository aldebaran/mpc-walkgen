#include <mpc-walkgen/qp-solvers/qpoases-solver.h>


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

void QPOasesSolver::solve(MPCSolution & result){


	qp_ = new qpOASES::QProblem(nbVar_, nbCtr_);
	qp_->setPrintLevel(qpOASES::PL_NONE);

	reorderInitialSolution(result);

	qpOASES::Constraints* ctrInit = new  qpOASES::Constraints(nbCtr_);
	qpOASES::Bounds* boundsInit = new  qpOASES::Bounds(nbVar_);
	if (result.useWarmStart){
		result.qpSolution = result.initialSolution;
		result.constraints = result.initialConstraints;
		for(int i=0;i<nbVar_;++i){
		      if (result.constraints(i)==0){
			    boundsInit->setupBound(i,qpOASES::ST_INACTIVE);
		      }else if (result.constraints(i)==1){
			   boundsInit->setupBound(i,qpOASES::ST_LOWER);
		      }else{
			   boundsInit->setupBound(i,qpOASES::ST_UPPER);
		      }
		}
		for(int i=0;i<nbCtr_;++i){
		      if (result.constraints(nbVar_+i)==0){
			   ctrInit->setupConstraint(i,qpOASES::ST_INACTIVE);
		      }else if (result.constraints(nbVar_+i)==1){
			   ctrInit->setupConstraint(i,qpOASES::ST_LOWER);
		      }else{
			   ctrInit->setupConstraint(i,qpOASES::ST_UPPER);
		      }
		}

	}else{
		ctrInit->setupAllInactive();
		boundsInit->setupAllFree();
		if (result.qpSolution.rows()!=nbVar_){
			result.qpSolution.setZero(nbVar_);
		}else{
			result.qpSolution.fill(0);
		}
		if (result.constraints.rows()!=nbVar_ + nbCtr_){
			result.constraints.setZero(nbVar_ + nbCtr_);
		}else{
			result.constraints.fill(0);
		}
	}

	int ittMax=100;

	MatrixXd A = matrixA_().transpose();
	qp_->init(matrixQ_().data(), vectorP_().data(), A.data(),
		  vectorXL_().data(), vectorXU_().data(),
		  vectorBL_().data(), vectorBU_().data(),
		  ittMax, 0,
		  result.qpSolution.data(),NULL,
		  boundsInit, ctrInit);

        double* sol = new double[nbVar_];
        qp_->getPrimalSolution(sol);
        for(int i=0;i<nbVar_;++i){
            result.qpSolution(i) = sol[i];
        }

        qpOASES::Constraints ctr;
        qpOASES::Bounds bounds;
        qp_->getConstraints(ctr);
        qp_->getBounds(bounds);
        for(int i=0;i<nbVar_;++i){
            if (bounds.getStatus(i)==qpOASES::ST_LOWER){
                result.constraints(i)=1;
            }else if (bounds.getStatus(i)==qpOASES::ST_UPPER){
                result.constraints(i)=2;
            }else{
                result.constraints(i)=0;
            }
        }
        for(int i=0;i<nbCtr_;++i){
            if (ctr.getStatus(i)==qpOASES::ST_LOWER){
                result.constraints(i+nbVar_)=1;
            }else if (ctr.getStatus(i)==qpOASES::ST_UPPER){
                result.constraints(i+nbVar_)=2;
            }else{
                result.constraints(i+nbVar_)=0;
            }
        }

	reorderSolution(result);

	delete qp_;
	delete ctrInit;
	delete boundsInit;
}


bool QPOasesSolver::resizeAll(){
	bool maxSizechanged = QPSolver::resizeAll();

	return maxSizechanged;
}

