////////////////////////////////////////////////////////////////////////////////
///
///\file test-qpoases-solver.cpp
///\brief Test the QPOases solver
///\author Lafaye Jory
///\date 20/07/13
///
/// The purpose of this test is to
/// * check the QPSolvers return the expected results
/// * check that using all the QPSolver variants from the same binary works
///   (no symbol collision...) so we use makeQPSolver<float>,
///   makeQPSolver<double> and also the raw qpAOSESfloat library
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <boost/scoped_ptr.hpp>
#include "../src/type.h"
#include "../src/qpsolverfactory.h"

#define QPOASES_REAL_IS_FLOAT
#include <QProblem.hpp>

template <typename Scalar_>
class QPSolverTest: public ::testing::Test{};

typedef ::testing::Types<float, double> MyTypes;

TYPED_TEST_CASE(QPSolverTest, MyTypes);


TYPED_TEST(QPSolverTest, testSolver)
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

  Q(0,0)=5.f; Q(0,1)=4.f;
  Q(1,0)=4.f; Q(1,1)=5.f;

  p[0]=1.f; p[1]=-1.f;

  typename QPMatrices<TypeParam>::VectorX x(2);
  x[0]=-10.f;
  x[1]=-10.f;

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


TYPED_TEST(QPSolverTest, testSolverWithConstraint)
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

  Q(0,0)=5.f; Q(0,1)=4.f;
  Q(1,0)=4.f; Q(1,1)=5.f;

  p[0]=1.f; p[1]=-1.f;

  A(0,0)=1.f; A(0,1)=0.f;
  b(0)=-2.f;

  typename QPMatrices<TypeParam>::VectorX x(2);
  x[0]=-10.f;
  x[1]=-10.f;

  m.Q = Q;
  m.p = p;
  m.A = A;
  m.At = A.transpose();
  m.bl = bl;
  m.bu = b;
  m.xl = xl;
  m.xu = xu;

  qp->solve(m, x);

  ASSERT_NEAR(x(0), -2.0f, EPSILON);
  ASSERT_NEAR(x(1), 1.8f, EPSILON);
}


TEST(QPOasesTest, testSolverWithConstraint)
{
  using namespace MPCWalkgen;

  QPMatrices<float> m;

  QPMatrices<float>::MatrixX Q(2, 2);
  QPMatrices<float>::MatrixX A(1, 2);
  QPMatrices<float>::VectorX p(2);
  QPMatrices<float>::VectorX b(1);
  QPMatrices<float>::VectorX bl(1);
  bl.fill(-100);
  QPMatrices<float>::VectorX xu(2);
  xu.fill(100);
  QPMatrices<float>::VectorX xl(2);
  xl.fill(-100);

  Q(0,0)=5.f; Q(0,1)=4.f;
  Q(1,0)=4.f; Q(1,1)=5.f;

  p[0]=1.f; p[1]=-1.f;

  A(0,0)=1.f; A(0,1)=0.f;
  b(0)=-2.f;

  QPMatrices<float>::VectorX x(2);
  x[0]=-10.f;
  x[1]=-10.f;

  m.Q = Q;
  m.p = p;
  m.A = A;
  m.At = A.transpose();
  m.bl = bl;
  m.bu = b;
  m.xl = xl;
  m.xu = xu;

  boost::scoped_ptr<qpOASES::QProblem> qpRaw(new qpOASES::QProblem(2, 1));
  qpRaw->setPrintLevel(qpOASES::PL_NONE);
  int ittMax = 10000;
  qpRaw->init(Q.data(),
              p.data(),
              m.At.data(),
              m.xl.data(),
              m.xu.data(),
              m.bl.data(),
              m.bu.data(),
              ittMax,
              NULL);
  qpRaw->getPrimalSolution(x.data());
  ASSERT_NEAR(x(0), -2.0f, EPSILON);
  ASSERT_NEAR(x(1), 1.8f, EPSILON);
}
