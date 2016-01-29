#pragma once
#ifndef MPC_WALKGEN_QPSOLVER_SRC_QPOASES_HXX
#define MPC_WALKGEN_QPSOLVER_SRC_QPOASES_HXX

#include <mpc-walkgen/qpsolver.h>
#include <QProblem.hpp>
#include <iostream>

using namespace MPCWalkgen;

template <typename Scalar>
class QPOasesSolver : public QPSolver<Scalar>
{
public:
  QPOasesSolver(int nbVar, int nbCtr);

  bool solve(const QPMatrices<Scalar>& m,
             typename QPMatrices<Scalar>::VectorX& sol,
             bool useWarmStart = false);

  inline int getNbVar() const
  {return nbVar_;}

  inline int getNbCtr() const
  {return nbCtr_;}

private:
  Eigen::VectorXi constraints_;
  int nbVar_;
  int nbCtr_;
  ::qpOASES::QProblem qp_;

  bool qpIsInitialized_;
};

template <typename Scalar>
QPOasesSolver<Scalar>::QPOasesSolver(int nbVar, int nbCtr)
:constraints_(nbVar+nbCtr)
,nbVar_(nbVar)
,nbCtr_(nbCtr)
,qp_(nbVar, nbCtr)
,qpIsInitialized_(false)
{
  constraints_.fill(0);
  qp_.setPrintLevel(qpOASES::PL_NONE);
}

template <typename Scalar>
bool QPOasesSolver<Scalar>::solve(const QPMatrices<Scalar>& m,
                                  typename QPMatrices<Scalar>::VectorX& sol,
                                  bool useWarmStart)
{
  assert(m.Q.rows() == m.Q.cols());
  assert(m.Q.rows() == m.p.size());
  assert(m.Q.rows() == m.A.cols());
  assert(m.A.rows() == m.bl.rows());
  assert(m.A.rows() == m.bu.rows());
  assert(m.Q.rows() == m.xl.rows());
  assert(m.Q.rows() == m.xu.rows());
  assert(m.Q.rows() == sol.size());
  assert(m.Q.rows() == nbVar_);
  assert(m.A.rows() == nbCtr_);

  qp_.setPrintLevel(qpOASES::PL_NONE);

  //The number of iterations can be high in the init phase (approximatively equals to the
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
    ret = qp_.init(m.Q.data(), m.p.data(), m.A.data(),
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

#endif //MPC_WALKGEN_QPSOLVER_SRC_QPOASES_HXX
