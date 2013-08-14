////////////////////////////////////////////////////////////////////////////////
///
///\file test-qpoases-solver.cpp
///\brief Test the QPOases solver
///\author Lafaye Jory
///\date 20/07/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/solver/qpoases_solver.h"

class QPOasesTest: public ::testing::Test{};



TEST_F(QPOasesTest, testSolver)
{
  using namespace MPCWalkgen;

  QPOasesSolver qp(2, 0);
  QPMatrices m;

  MatrixX Q(2, 2);
  MatrixX A(0, 2);
  VectorX p(2);
  VectorX b(0, 1);
  VectorX bl(0, 1);
  VectorX xu(2);
  xu.fill(100);
  VectorX xl(2);
  xl.fill(-100);

  Q(0,0)=5;Q(0,1)=4;
  Q(1,0)=4;Q(1,1)=5;

  p[0]=1;p[1]=-1;

  VectorX x(2);
  x[0]=-10;
  x[1]=-10;

  m.Q = Q;
  m.p = p;
  m.A = A;
  m.At = A.transpose();
  m.bl = bl;
  m.bu = b;
  m.xl = xl;
  m.xu = xu;

  qp.solve(m, x);

  ASSERT_NEAR(x(0), -1, EPSILON);
  ASSERT_NEAR(x(1), 1, EPSILON);
}


TEST_F(QPOasesTest, testSolverWithConstraint)
{
  using namespace MPCWalkgen;

  QPOasesSolver qp(2, 1);
  QPMatrices m;

  MatrixX Q(2, 2);
  MatrixX A(1, 2);
  VectorX p(2);
  VectorX b(1);
  VectorX bl(1);
  bl.fill(-100);
  VectorX xu(2);
  xu.fill(100);
  VectorX xl(2);
  xl.fill(-100);

  Q(0,0)=5;Q(0,1)=4;
  Q(1,0)=4;Q(1,1)=5;

  p[0]=1;p[1]=-1;

  A(0,0)=1;A(0,1)=0;
  b(0)=-2;

  VectorX x(2);
  x[0]=-10;
  x[1]=-10;

  m.Q = Q;
  m.p = p;
  m.A = A;
  m.At = A.transpose();
  m.bl = bl;
  m.bu = b;
  m.xl = xl;
  m.xu = xu;

  qp.solve(m, x);

  ASSERT_NEAR(x(0), -2.0, EPSILON);
  ASSERT_NEAR(x(1), 1.8, EPSILON);
}
