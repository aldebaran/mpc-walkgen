
#include <cmath>
#include <cstdio>
#include <iostream>
#include <cstring>

#include <../src/common/qp-solvers/qpoases-solver.h>
#include <../src/humanoid/types.h>
#include <../src/common/qp-solvers/lssol-solver.h>
using namespace Eigen;
using namespace MPCWalkgen;


int main ()
{
  int nbVar=5;
  int nbCtr=3;

  LSSOLSolver qp1(nbVar,nbCtr);
  QPOasesSolver qp2(nbVar,nbCtr);


  qp1.reset();
  qp1.nbVar(nbVar);
  qp1.nbCtr(nbCtr);

  qp2.reset();
  qp2.nbVar(nbVar);
  qp2.nbCtr(nbCtr);

  // create the qp problem
  MatrixXd Q(nbVar,nbVar);
  Q << 1,3,2,-4.6,-1.5  ,  3,2.5,6,4,8  ,  -5,-6,-4.2,-45,12  ,  -15,-12.87,0.025,0.154,1  ,  0,0,0,0,1;
  qp1.matrix(matrixQ).addTerm(Q.transpose()*Q);
  qp2.matrix(matrixQ).addTerm(Q.transpose()*Q);

  VectorXd P(nbVar);
  P << -2, -6, 0.5, -1.5, 8;
  qp1.vector(vectorP).addTerm(P);
  qp2.vector(vectorP).addTerm(P);

  MatrixXd A(nbCtr,nbVar);
  A << 1,  1, 0.5, 0, 0,
    -1,  2, -2, -1.5, 0,
     2,  1, 0, 3, -4.5;
  qp1.matrix(matrixA).addTerm(A);
  qp2.matrix(matrixA).addTerm(A);

  VectorXd bl(nbCtr);
  bl << -4,0,-5;
  qp1.vector(vectorBL).addTerm(bl);
  qp2.vector(vectorBL).addTerm(bl);

  VectorXd bu(nbCtr);
  bu << -2,2,3;
  qp1.vector(vectorBU).addTerm(bu);
  qp2.vector(vectorBU).addTerm(bu);

  VectorXd xl(nbVar);
  xl.fill(-2);
  qp1.vector(vectorXL).addTerm(xl);
  qp2.vector(vectorXL).addTerm(xl);

  VectorXd xu(nbVar);
  xu.fill(4);
  qp1.vector(vectorXU).addTerm(xu);
  qp2.vector(vectorXU).addTerm(xu);

  MPCSolution result1, result2;
  result1.reset();
  result1.useWarmStart=false;
  result1.initialSolution.resize(nbVar);
  result1.initialConstraints.resize(nbVar+nbCtr);
  result1.reset();
  result1.useWarmStart=false;
  result1.initialSolution.resize(nbVar);
  result1.initialConstraints.resize(nbVar+nbCtr);

  result2.reset();
  result2.useWarmStart=false;
  result2.initialSolution.resize(nbVar);
  result2.initialConstraints.resize(nbVar+nbCtr);
  result2.reset();
  result2.useWarmStart=false;
  result2.initialSolution.resize(nbVar);
  result2.initialConstraints.resize(nbVar+nbCtr);

  qp1.solve(result1.qpSolution, result1.constraints,
     result1.initialSolution, result1.initialConstraints, result1.useWarmStart);
  qp2.solve(result2.qpSolution, result2.constraints,
     result2.initialSolution, result2.initialConstraints, result2.useWarmStart);

  std::cout << "Solution LSSOL   : " << result1.qpSolution.transpose() << std::endl;
  std::cout << "Solution QPOASES : " << result2.qpSolution.transpose() << std::endl;

  bool success = ((result1.qpSolution - result2.qpSolution).norm() < 1e-5);
  return (success ? 0 : 1);
}
