#include "qpoases_solver.h"


using namespace MPCWalkgen;

QPOasesSolver::QPOasesSolver()
:constraints_(2)
,qp_(1, 1)
,nbVar_(1)
,nbCtr_(1)
,qpIsInitialized_(false)
{
  constraints_.fill(0);
  qp_.setPrintLevel(qpOASES::PL_NONE);
}

QPOasesSolver::QPOasesSolver(int nbVar, int nbCtr)
:constraints_(nbVar+nbCtr)
,qp_(nbVar, nbCtr)
,nbVar_(nbVar)
,nbCtr_(nbCtr)
,qpIsInitialized_(false)
{
  constraints_.fill(0);
  qp_.setPrintLevel(qpOASES::PL_NONE);
}

bool QPOasesSolver::solve(const QPMatrices& m, VectorX& sol,
                          bool useWarmStart)
{
  assert(m.Q.rows() == m.Q.cols());
  assert(m.Q.rows() == m.p.size());
  assert(m.Q.rows() == m.A.cols());
  assert(m.Q.rows() == m.At.rows());
  assert(m.A.rows() == m.bl.rows());
  assert(m.A.rows() == m.bu.rows());
  assert(m.At.cols() == m.bl.rows());
  assert(m.At.cols() == m.bu.rows());
  assert(m.Q.rows() == m.xl.rows());
  assert(m.Q.rows() == m.xu.rows());
  assert(m.Q.rows() == sol.size());
  assert(m.Q.rows() == nbVar_);
  assert(m.A.rows() == nbCtr_);

  qp_.setPrintLevel(qpOASES::PL_NONE);


  //The number of itterations can be high in the init phase (approximatively equals to the
  //number of constraints, aka 250).
  int ittMax = 10000;
  ::qpOASES::returnValue ret;
  if (qpIsInitialized_ && useWarmStart)
  {
    ret = qp_.hotstart(m.p.data(), m.xl.data(), m.xu.data(),
                                            m.bl.data(), m.bu.data(),
                                            ittMax, 0);
  }
  else
  {
    ret = qp_.init(m.Q.data(), m.p.data(), m.At.data(),
                                       m.xl.data(), m.xu.data(), m.bl.data(), m.bu.data(),
                                       ittMax, 0);
    qpIsInitialized_ = true;
  }
  qp_.getPrimalSolution(sol.data());

  if (ret!=::qpOASES::SUCCESSFUL_RETURN){
    std::cout << "[ERROR] MPC-Walkgen infeasible. QPOases error code " << ret << std::endl;
    return false;
  }

  return true;
}

