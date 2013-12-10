////////////////////////////////////////////////////////////////////////////////
///
///\file test-qpoases-solver.cpp
///\brief Test the QPOases solver
///\author Lafaye Jory
///\date 20/07/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <boost/scoped_ptr.hpp>
#include "../src/type.h"
#include "../src/qpsolverfactory.h"

template <typename Scalar_>
class QPOasesTest: public ::testing::Test{};

typedef ::testing::Types<float, double> MyTypes;

TYPED_TEST_CASE(QPOasesTest, MyTypes);


TYPED_TEST(QPOasesTest, testSolver)
{
  using namespace MPCWalkgen;

  boost::scoped_ptr< QPSolver<TypeParam> > qp(makeQPSolver<TypeParam>(2, 0));
  QPMatrices<TypeParam> m;

  typename QPMatrices<TypeParam>::MatrixX Q(2, 2);
  typename QPMatrices<TypeParam>::MatrixX A(0, 2);
  typename QPMatrices<TypeParam>::VectorX p(2);
  typename QPMatrices<TypeParam>::VectorX b(0, 1);
  typename QPMatrices<TypeParam>::VectorX bl(0, 1);
  typename QPMatrices<TypeParam>::VectorX xu(2);
  xu.fill(100);
  typename QPMatrices<TypeParam>::VectorX xl(2);
  xl.fill(-100);

  Q(0,0)=5;Q(0,1)=4;
  Q(1,0)=4;Q(1,1)=5;

  p[0]=1;p[1]=-1;

  typename QPMatrices<TypeParam>::VectorX x(2);
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

  qp->solve(m, x);

  ASSERT_NEAR(x(0), -1, EPSILON);
  ASSERT_NEAR(x(1), 1, EPSILON);
}


TYPED_TEST(QPOasesTest, testSolverWithConstraint)
{
  using namespace MPCWalkgen;

  boost::scoped_ptr< QPSolver<TypeParam> > qp(makeQPSolver<TypeParam>(2, 1));
  QPMatrices<TypeParam> m;

  typename QPMatrices<TypeParam>::MatrixX Q(2, 2);
  typename QPMatrices<TypeParam>::MatrixX A(1, 2);
  typename QPMatrices<TypeParam>::VectorX p(2);
  typename QPMatrices<TypeParam>::VectorX b(1);
  typename QPMatrices<TypeParam>::VectorX bl(1);
  bl.fill(-100);
  typename QPMatrices<TypeParam>::VectorX xu(2);
  xu.fill(100);
  typename QPMatrices<TypeParam>::VectorX xl(2);
  xl.fill(-100);

  Q(0,0)=5;Q(0,1)=4;
  Q(1,0)=4;Q(1,1)=5;

  p[0]=1;p[1]=-1;

  A(0,0)=1;A(0,1)=0;
  b(0)=-2;

  typename QPMatrices<TypeParam>::VectorX x(2);
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

  qp->solve(m, x);

  ASSERT_NEAR(x(0), -2.0, EPSILON);
  ASSERT_NEAR(x(1), 1.8, EPSILON);
}
