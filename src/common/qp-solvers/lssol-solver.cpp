#include "lssol-solver.h"

#include <lssol/lssol.h>

using namespace MPCWalkgen;
using namespace Eigen;


LSSOLSolver::LSSOLSolver(const int nbVarMin, const int nbCtrMin, const int nbVarMax, const int nbCtrMax)
  :QPSolver(nbVarMin, nbCtrMin, nbVarMax, nbCtrMax)
  ,kx_(nbVarMax)
  ,bb_(1)
  ,lambda_(nbVarMax+nbCtrMax)
  ,bu_(nbVarMax+nbCtrMax)
  ,bl_(nbVarMax+nbCtrMax)
  ,leniw_(nbVarMax)
  ,lenw_(2*nbVarMax*nbVarMax + 10*nbVarMax+ 6*nbCtrMax)
  ,war_(lenw_)
  ,iwar_(leniw_)
  ,inform_(0)
  ,iter_(0)
  ,obj_(0)
  ,useCholesky_(true)
{
  sendOption("Print Level = 0");
  sendOption("Problem Type = QP4");
  sendOption("warm start");
}

LSSOLSolver::~LSSOLSolver(){}

bool LSSOLSolver::solve(VectorXd & qpSolution,
      VectorXi & constraints,
      VectorXd & initialSolution,
      VectorXi & initialConstraints,
      bool useWarmStart){

  reorderInitialSolution(initialSolution, initialConstraints);


  // Pile up XL and BL
  bl_.segment(0,      nbVar_) = vectorXL_().block(0,0,nbVar_,1);
  bl_.segment(nbVar_, nbCtr_) = vectorBL_().block(0,0,nbCtr_,1);

  // Pile up XU and BU
  bu_.segment(0,      nbVar_) = vectorXU_().block(0,0,nbVar_,1);
  bu_.segment(nbVar_, nbCtr_) = vectorBU_().block(0,0,nbCtr_,1);

  if (useWarmStart){
    qpSolution = initialSolution;
    constraints = initialConstraints;
  }else{
    if (qpSolution.rows() != nbVar_){
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

  for(int i=0;i<nbVar_;++i){
    kx_(i)=i+1;
  }

  // The error 6 of lssol "An input parameter is invalid" may be
  // difficult to debug. The following tests can help.
  assert(bl_.size() >= nbVar_ + nbCtr_);
  assert(bu_.size() >= nbVar_ + nbCtr_);
  assert(vectorP_().size() >= nbVar_);
  assert(constraints.size() >= nbVar_ + nbCtr_);
  assert(leniw_>=nbVar_);
  assert((nbCtr_ > 0)  || (lenw_ >=10*nbVar_));
  assert((nbCtr_ == 0) || (lenw_ >=2*nbVar_*nbVar_ + 10*nbVar_+ 6*nbCtr_));


  lssol_(&nbVar_, &nbVar_,
      &nbCtr_, &nbCtr_, &nbVar_,
      matrixA_().data(), bl_.data(), bu_.data(),vectorP_().data(),
      constraints.data(), kx_.data(), qpSolution.data(),
      matrixQ_.cholesky().data(), bb_.data(), &inform_, &iter_, &obj_, lambda_.data(),
      iwar_.data(), &leniw_, war_.data(), &lenw_);

  if (inform_==3){
    std::cout << "LSSOL : No feasible point was found" << std::endl;
  }

  reorderSolution(qpSolution, constraints, initialConstraints);

  return true;
}


bool LSSOLSolver::resizeAll(){
  bool maxSizechanged = QPSolver::resizeAll();

  if (maxSizechanged){
    kx_.resize(nbVarMax_);
    lambda_.resize(nbVarMax_+nbCtrMax_);
    bu_.resize(nbVarMax_+nbCtrMax_);
    bl_.resize(nbVarMax_+nbCtrMax_);
    leniw_=nbVarMax_;
    lenw_=2*nbVarMax_*nbVarMax_ + 10*nbVarMax_+ 6*nbCtrMax_;
    war_.resize(lenw_);
    iwar_.resize(leniw_);
  }

  return maxSizechanged;
}

