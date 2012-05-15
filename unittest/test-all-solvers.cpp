
#include <cmath>
#include <cstdio>
#include <iostream>
#include <cstring>

#include "../src/humanoid/types.h"
#include "../src/common/qp-solver.h"
#include <mpc-walkgen/common/qp-solver-type.h>

using namespace Eigen;
using namespace MPCWalkgen;
using namespace Humanoid;

Eigen::VectorXd test_all_solvers(QPSolver & qp1, int nbVar, int nbCtr)
{
  qp1.reset();
  qp1.nbVar(nbVar);
  qp1.nbCtr(nbCtr);

  // create the qp problem
  MatrixXd Q(nbVar,nbVar);
  Q << 1,3,2,-4.6,-1.5  ,  3,2.5,6,4,8  ,  -5,-6,-4.2,-45,12  ,  -15,-12.87,0.025,0.154,1  ,  0,0,0,0,1;
  qp1.matrix(matrixQ).addTerm(Q.transpose()*Q);

  VectorXd P(nbVar);
  P << -2, -6, 0.5, -1.5, 8;
  qp1.vector(vectorP).addTerm(P);

  MatrixXd A(nbCtr,nbVar);
  A << 1,  1, 0.5, 0, 0,
    -1,  2, -2, -1.5, 0,
     2,  1, 0, 3, -4.5;
  qp1.matrix(matrixA).addTerm(A);

  VectorXd bl(nbCtr);
  bl << -4,0,-5;
  qp1.vector(vectorBL).addTerm(bl);

  VectorXd bu(nbCtr);
  bu << -2,2,3;
  qp1.vector(vectorBU).addTerm(bu);

  VectorXd xl(nbVar);
  xl.fill(-2);
  qp1.vector(vectorXL).addTerm(xl);

  VectorXd xu(nbVar);
  xu.fill(4);
  qp1.vector(vectorXU).addTerm(xu);

  MPCSolution result1;
  result1.reset();
  result1.useWarmStart=false;
  result1.initialSolution.resize(nbVar);
  result1.initialConstraints.resize(nbVar+nbCtr);

  qp1.solve(result1.qpSolution, result1.constraints,
     result1.initialSolution, result1.initialConstraints, result1.useWarmStart);

  return result1.qpSolution;
}


int main()
{
  bool success = true;

  int nbVar=5;
  int nbCtr=3;

  Eigen::VectorXd solution(nbVar);
  solution << 0.404315842674, -1.40431584267,       -2, 0.524701647986, 0.791078367425;

#ifdef USE_QPOASES
  std::cout << "bench-qpsolver test qpOASES " << std::endl;
  QPSolver * qp1 = createQPSolver(QPSOLVERTYPE_QPOASES, nbVar, nbCtr);
  Eigen::VectorXd qp1Solution = test_all_solvers(*qp1, nbVar, nbCtr);
  bool success1 = ((qp1Solution - solution).norm() < 1e-5);
  std::cout << "Solution QPOASES ("<< success1 <<"): " << qp1Solution.transpose() << std::endl;
  if (qp1) {
    delete qp1;
  }
  success = success1 && success;
#endif //USE_QPOASES

#ifdef USE_LSSOL
  std::cout << "bench-qpsolver test LSSOL " << std::endl;
  QPSolver * qp2 = createQPSolver(QPSOLVERTYPE_LSSOL, nbVar, nbCtr);
  Eigen::VectorXd qp2Solution = test_all_solvers(*qp2, nbVar, nbCtr);
  bool success2 = ((qp2Solution - solution).norm() < 1e-5);
  std::cout << "Solution LSSOL ("<< success2 <<") : " << qp2Solution.transpose() << std::endl;
  if (qp2) {
    delete qp2;
  }
  success = success2 && success;
#endif //USE_LSSOL

  return (success ? 0 : 1);
}
