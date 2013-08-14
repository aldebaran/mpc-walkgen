////////////////////////////////////////////////////////////////////////////////
///
///\file test-zebulon-base-motion-constraint.cpp
///\brief Test the Zebulon base motion constraint function
///\author Lafaye Jory
///\date 20/07/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/model/zebulon_base_model.h"
#include "../src/function/zebulon_base_motion_constraint.h"

class ZebulonBaseMotionConstraintTest: public ::testing::Test{};



TEST_F(ZebulonBaseMotionConstraintTest, functionValue)
{
  using namespace MPCWalkgen;

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


TEST_F(ZebulonBaseMotionConstraintTest, sizeOfvalues)
{
  using namespace MPCWalkgen;

  int nbSamples = 3;
  Scalar samplingPeriod = 1.0;
  bool autoCompute = true;
  BaseModel m(nbSamples, samplingPeriod, autoCompute);

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
