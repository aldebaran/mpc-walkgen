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
             int maxNbIterations,
             typename QPMatrices<Scalar>::VectorX& sol,
             bool useWarmStart = false,
             std::ostream *os = nullptr);

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
                                  int maxNbIterations,
                                  typename QPMatrices<Scalar>::VectorX& sol,
                                  bool useWarmStart,
                                  std::ostream *os)
{
  assert(m.dimensionsAreConsistent(nbVar_, nbCtr_));

  qp_.setPrintLevel(qpOASES::PL_NONE);

  ::qpOASES::returnValue ret;
  const bool doWarmStart = qpIsInitialized_ && useWarmStart;
  if (doWarmStart)
  {
    ret = qp_.hotstart(m.p.data(), m.xl.data(), m.xu.data(), m.bl.data(),
                       m.bu.data(), maxNbIterations, 0);
  }
  else
  {
    ret = qp_.init(m.Q.data(), m.p.data(), m.A.data(),
                   m.xl.data(), m.xu.data(), m.bl.data(), m.bu.data(),
                   maxNbIterations, 0);
    qpIsInitialized_ = true;
  }
  // we write the result even if qpOASES did report an error:
  // the "non-solution" might be useful: it is optimal but unfeasible with
  // respect to some constraints.
  qp_.getPrimalSolution(sol.data());
  if ((ret != ::qpOASES::SUCCESSFUL_RETURN) && (os != nullptr))
  {
    *os << "QP unfeasible. QPOases error code: " << ret
        << ". Number of working set recalculations: " << maxNbIterations
        << ". Was warm start: " << doWarmStart << ".\n";
    return false;
  }

  return true;
}

#endif //MPC_WALKGEN_QPSOLVER_SRC_QPOASES_HXX
