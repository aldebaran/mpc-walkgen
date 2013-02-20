#include "qpoases-solver.h"



using namespace MPCWalkgen;
using namespace Eigen;

QPOasesSolver::QPOasesSolver(const int nbVarMin, const int nbCtrMin, const int nbVarMax, const int nbCtrMax)
  :QPSolver(nbVarMin, nbCtrMin, nbVarMax, nbCtrMax)
  ,nbVarMin_(nbVarMin)
  ,nbCtrMin_(nbCtrMin)
  ,sizeNbCtr_(nbCtrMax-nbCtrMin+1)
  ,sizeNbVar_(nbVarMax-nbVarMin+1)
{
  // If one of the new throws (getting out of memory for instance)
  // All the memory allocated by the previous new will be leaked.
  // We could fix this by catching the exception, freeing the memory,
  // the re-throwing, or by using RAII (with boost::scoped_ptr for instance)
  for(int i=0;i<sizeNbVar_;++i){
    for(int j=0;j<sizeNbCtr_;++j){
      qp_.push_back(new qpOASES::QProblem(i+nbVarMin, j+nbCtrMin));
      qp_.back()->setPrintLevel(qpOASES::PL_NONE);
    }
  }

  for(int i=0;i<sizeNbCtr_;++i){
    ctrInit_.push_back(new qpOASES::Constraints(i+nbCtrMin));
  }

  for(int i=0;i<sizeNbVar_;++i){
    boundsInit_.push_back(new qpOASES::Bounds(i+nbVarMin));
  }
}

QPOasesSolver::~QPOasesSolver(){
  for(unsigned i=0;i<qp_.size();++i){
    delete qp_[i];
  }

  for(unsigned i=0;i<ctrInit_.size();++i){
    delete ctrInit_[i];
  }

  for(unsigned i=0;i<boundsInit_.size();++i){
    delete boundsInit_[i];
  }
}

bool QPOasesSolver::solve(VectorXd & qpSolution,
        VectorXi & constraints,
        VectorXd & initialSolution,
        VectorXi & initialConstraints,
        bool useWarmStart){
  int varNumber = nbVar_- nbVarMin_;
  int ctrNumber = nbCtr_ - nbCtrMin_;
  int qpNumber = varNumber*sizeNbCtr_ + ctrNumber;

  reorderInitialSolution(initialSolution, initialConstraints);

  if (useWarmStart){
    qpSolution = initialSolution;
    constraints = initialConstraints;
    for(int i=0;i<nbVar_;++i){
          if (constraints(i)==0){
         boundsInit_[varNumber]->setupBound(i,qpOASES::ST_INACTIVE);
          }else if (constraints(i)==1){
         boundsInit_[varNumber]->setupBound(i,qpOASES::ST_LOWER);
          }else{
         boundsInit_[varNumber]->setupBound(i,qpOASES::ST_UPPER);
          }
    }
    for(int i=0;i<nbCtr_;++i){
          if (constraints(nbVar_+i)==0){
         ctrInit_[ctrNumber]->setupConstraint(i,qpOASES::ST_INACTIVE);
          }else if (constraints(nbVar_+i)==1){
         ctrInit_[ctrNumber]->setupConstraint(i,qpOASES::ST_LOWER);
          }else{
         ctrInit_[ctrNumber]->setupConstraint(i,qpOASES::ST_UPPER);
          }
    }

  }else{
    ctrInit_[ctrNumber]->setupAllInactive();
    boundsInit_[varNumber]->setupAllFree();
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

  qp_[qpNumber]->init(matrixQ_().data(), vectorP_().data(), A.data(),
      vectorXL_().data(), vectorXU_().data(),
      vectorBL_().data(), vectorBU_().data(),
      ittMax, 0,
      qpSolution.data(),NULL,
      boundsInit_[varNumber], ctrInit_[ctrNumber]);


        ::qpOASES::returnValue ret = qp_[qpNumber]->getPrimalSolution(qpSolution.data());

        qpOASES::Constraints ctr;
        qpOASES::Bounds bounds;
        qp_[qpNumber]->getConstraints(ctr);
        qp_[qpNumber]->getBounds(bounds);
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


        if (ret!=::qpOASES::SUCCESSFUL_RETURN){
          std::cout << "[ERROR] MPC-Walkgen infeasible" << std::endl;
          return false;
        }
        return true;
}


bool QPOasesSolver::resizeAll(){
  bool maxSizechanged = QPSolver::resizeAll();

  return maxSizechanged;
}

