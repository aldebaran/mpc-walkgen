
#include <gtest/gtest.h>

#include "../../src/common/interpolation.h"
#include "../../src/common/tools.h"

using namespace Eigen;
using namespace MPCWalkgen;

class InterpolationTest: public ::testing::Test
{
  protected:

    virtual void SetUp(){
      T = 10;
      EPSILON = 1e-4;
      initialstate << 1, 2, 3;
      finalState << 4, 5, 6;
      factor.resize(12);
      interpolation.computePolynomialNormalisedFactors(factor, initialstate, finalState, T);
    }

    MPCWalkgen::Interpolation interpolation;
    double T;
    double t;
    double EPSILON;
    Eigen::Vector3d initialstate;
    Eigen::Vector3d finalState;
    Eigen::VectorXd factor;
    Eigen::Vector4d subfactor;
    Eigen::Vector4d subfactor1;
    Eigen::Vector4d subfactor2;
};


TEST_F(InterpolationTest, initialConstraints)
{
  t = 0.;

  interpolation.selectFactors(subfactor, factor, T/6, T);

  ASSERT_NEAR (p(subfactor, t/T), initialstate[0], EPSILON);
  ASSERT_NEAR (dp(subfactor, t/T)/T, initialstate[1], EPSILON);
  ASSERT_NEAR (ddp(subfactor, t/T)/(T*T), initialstate[2], EPSILON);
}


TEST_F(InterpolationTest, finalConstraints)
{
  t = T;

  interpolation.selectFactors(subfactor, factor, 5*T/6, T);

  ASSERT_NEAR (p(subfactor, t/T), finalState[0], EPSILON);
  ASSERT_NEAR (dp(subfactor, t/T)/T, finalState[1], EPSILON);
  ASSERT_NEAR (ddp(subfactor, t/T)/(T*T), finalState[2], EPSILON);
}


TEST_F(InterpolationTest, firstJunction)
{
  t = T/3;

  interpolation.selectFactors(subfactor1, factor, T/6, T);
  interpolation.selectFactors(subfactor2, factor, T/2, T);

  ASSERT_NEAR (p(subfactor1, t/T), p(subfactor2, t/T), EPSILON);
  ASSERT_NEAR (dp(subfactor1, t/T)/T, dp(subfactor2, t/T)/T, EPSILON);
  ASSERT_NEAR (ddp(subfactor1, t/T)/(T*T), ddp(subfactor2, t/T)/(T*T), EPSILON);
}


TEST_F(InterpolationTest, secondJunction)
{
  t = 2*T/3;

  interpolation.selectFactors(subfactor1, factor, T/2, T);
  interpolation.selectFactors(subfactor2, factor, 5*T/6, T);

  ASSERT_NEAR (p(subfactor1, t/T), p(subfactor2, t/T), EPSILON);
  ASSERT_NEAR (dp(subfactor1, t/T)/T, dp(subfactor2, t/T)/T, EPSILON);
  ASSERT_NEAR (ddp(subfactor1, t/T)/(T*T), ddp(subfactor2, t/T)/(T*T), EPSILON);
}


TEST_F(InterpolationTest, firstPolynomValue)
{
  t = T/6;

  interpolation.selectFactors(subfactor, factor, T/6, T);

  ASSERT_NEAR (p(subfactor, t/T), 7.12731, EPSILON);
  ASSERT_NEAR (dp(subfactor, t/T), 45.2917, EPSILON);
  ASSERT_NEAR (ddp(subfactor, t/T), 3.5, EPSILON);
}


TEST_F(InterpolationTest, secondPolynomValue)
{
  t = T/2;

  interpolation.selectFactors(subfactor, factor, T/2, T);

  ASSERT_NEAR (p(subfactor, t/T), 13.3333, EPSILON);
  ASSERT_NEAR (dp(subfactor, t/T), -18.25, EPSILON);
  ASSERT_NEAR (ddp(subfactor, t/T), -180, EPSILON);
}


TEST_F(InterpolationTest, thirdPolynomValue)
{
  t = 5*T/6;

  interpolation.selectFactors(subfactor, factor, 5*T/6, T);

  ASSERT_NEAR (p(subfactor, t/T), 2.45602, EPSILON);
  ASSERT_NEAR (dp(subfactor, t/T), -22.2083, EPSILON);
  ASSERT_NEAR (ddp(subfactor, t/T), 266.5, EPSILON);
}

