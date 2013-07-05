#include <gtest/gtest.h>
#include "../src/model/zebulon_base_model.h"
#include "../src/function/zebulon_base_motion_constraint.h"

using namespace Eigen;
using namespace MPCWalkgen;

class BaseMotionConstraintTest: public ::testing::Test{};



TEST_F(BaseMotionConstraintTest, functionValue)
{
  BaseModel m;

  BaseMotionConstraint ctr(m);

  VectorX jerkInit(2);
  jerkInit(0) = 0.0;
  jerkInit(1) = 0.0;

  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(0)>0.0);
  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(1)>0.0);
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(0)<0.0);
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(1)<0.0);

  jerkInit(0) = 0.0;
  jerkInit(1) = 2.0;
  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(0)>0.0);
  ASSERT_NEAR(ctr.getFunctionSup(jerkInit)(1), 0.0, EPSILON);

  jerkInit(0) = 0.0;
  jerkInit(1) = 2.1;
  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(0)>0.0);
  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(1)<0.0);

  jerkInit(0) = 0.0;
  jerkInit(1) = -2.0;
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(0)<0.0);
  ASSERT_NEAR(ctr.getFunctionInf(jerkInit)(1), 0.0, EPSILON);


  jerkInit(0) = 0.0;
  jerkInit(1) = -2.1;
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(0)<0.0);
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(1)>0.0);

  jerkInit(0) = 2.0;
  jerkInit(1) = 0.0;
  ASSERT_NEAR(ctr.getFunctionSup(jerkInit)(0), 0.0, EPSILON);
  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(1)>0.0);

  jerkInit(0) = 2.1;
  jerkInit(1) = 0.0;
  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(0)<0.0);
  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(1)>0.0);

  jerkInit(0) = -2.0;
  jerkInit(1) = 0.0;
  ASSERT_NEAR(ctr.getFunctionInf(jerkInit)(0), 0.0, EPSILON);
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(1)<0.0);


  jerkInit(0) = -2.1;
  jerkInit(1) = 0.0;
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(0)>0.0);
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(1)<0.0);

}


TEST_F(BaseMotionConstraintTest, sizeOfvalues)
{
  int nbSamples = 3;
  BaseModel m(nbSamples);

  BaseMotionConstraint ctr(m);

  int M = ctr.getNbConstraints();

  VectorX jerkInit(2*nbSamples);
  jerkInit.fill(0.0);

  ASSERT_EQ(ctr.getGradient().rows(), M);
  ASSERT_EQ(ctr.getGradient().cols(), 2*nbSamples);
  ASSERT_EQ(ctr.getFunctionInf(jerkInit).rows(), M);
  ASSERT_EQ(ctr.getFunctionInf(jerkInit).cols(), 1);
  ASSERT_EQ(ctr.getFunctionSup(jerkInit).rows(), M);
  ASSERT_EQ(ctr.getFunctionSup(jerkInit).cols(), 1);

}
