#include <mpc-walkgen/qp-solvers/qpoases-solver.h>

//#include <QProblem.hpp>
#include <lssol/lssol.h>
using namespace MPCWalkgen;
using namespace Eigen;


QPOasesSolver::QPOasesSolver(const int nbVarMax, const int nbCtrMax)
	:QPSolver(nbVarMax, nbCtrMax)
	,ittMax_(100)
{
}

QPOasesSolver::~QPOasesSolver(){}

void QPOasesSolver::solve(MPCSolution & result){

	reorderInitialSolution(result);

	if (result.useWarmStart){
		result.qpSolution = result.initialSolution;
		result.constraints = result.initialConstraints;
	}else{
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
/*
        fQP->init(matrixQ_.dense().data(), vectorP_.data(), matrixA_.dense().data(),
                  vectorXL_.dense().data(), vectorXU_.dense().data(),
                  vectorBL_.dense().data(), vectorBU_.dense().data(), ittMax_, 0
                  );
        f_CopReal = f_Com[0];

        fQP->getPrimalSolution(f_solution);
*/

	reorderSolution(result);
}


bool QPOasesSolver::resizeAll(){
	bool maxSizechanged = QPSolver::resizeAll();

	return maxSizechanged;
}

