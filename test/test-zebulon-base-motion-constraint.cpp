////////////////////////////////////////////////////////////////////////////////
///
///\file test-zebulon-base-motion-constraint.cpp
///\brief Test the Zebulon base motion constraint function
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include "mpc_walkgen_gtest.h"
#include <mpc-walkgen/model/zebulon_base_model.h>
#include <mpc-walkgen/function/zebulon_base_motion_constraint.h>

class ZebulonBaseMotionConstraintTest: public ::testing::Test{};

using namespace MPCWalkgen;

typedef float TypeParam;

TYPED_TEST(MpcWalkgenTest, functionValue)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam)

  BaseModel<TypeParam> m;
  BaseMotionConstraint<TypeParam> ctr(m);

  VectorX jerkInit(2);
  jerkInit(0) = 0.0f;
  jerkInit(1) = 0.0f;

  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(0)>0.0);
  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(1)>0.0);
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(0)<0.0);
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(1)<0.0);

  jerkInit(0) = 0.0f;
  jerkInit(1) = 2.0f;
  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(0)>0.0);
  ASSERT_NEAR(ctr.getFunctionSup(jerkInit)(1), 0.0, Constant<TypeParam>::EPSILON);

  jerkInit(0) = 0.0f;
  jerkInit(1) = 2.1f;
  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(0)>0.0);
  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(1)<0.0);

  jerkInit(0) = 0.0f;
  jerkInit(1) = -2.0f;
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(0)<0.0);
  ASSERT_NEAR(ctr.getFunctionInf(jerkInit)(1), 0.0, Constant<TypeParam>::EPSILON);


  jerkInit(0) = 0.0f;
  jerkInit(1) = -2.1f;
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(0)<0.0);
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(1)>0.0);

  jerkInit(0) = 2.0f;
  jerkInit(1) = 0.0f;
  ASSERT_NEAR(ctr.getFunctionSup(jerkInit)(0), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(1)>0.0);

  jerkInit(0) = 2.1f;
  jerkInit(1) = 0.0f;
  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(0)<0.0);
  ASSERT_TRUE(ctr.getFunctionSup(jerkInit)(1)>0.0);

  jerkInit(0) = -2.0f;
  jerkInit(1) = 0.0f;
  ASSERT_NEAR(ctr.getFunctionInf(jerkInit)(0), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(1)<0.0);


  jerkInit(0) = -2.1f;
  jerkInit(1) = 0.0f;
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(0)>0.0);
  ASSERT_TRUE(ctr.getFunctionInf(jerkInit)(1)<0.0);
}


TYPED_TEST(MpcWalkgenTest, sizeOfvalues)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);

  int nbSamples = 3;
  TypeParam samplingPeriod = 1.0f;
  bool autoCompute = true;
  BaseModel<TypeParam> m(nbSamples, samplingPeriod, autoCompute);

  BaseMotionConstraint<TypeParam> ctr(m);

  int M = ctr.getNbConstraints();

  VectorX jerkInit(2*nbSamples);
  jerkInit.fill(0.0f);

  ASSERT_EQ(ctr.getGradient().rows(), M);
  ASSERT_EQ(ctr.getGradient().cols(), 2*nbSamples);
  ASSERT_EQ(ctr.getFunctionInf(jerkInit).rows(), M);
  ASSERT_EQ(ctr.getFunctionInf(jerkInit).cols(), 1);
  ASSERT_EQ(ctr.getFunctionSup(jerkInit).rows(), M);
  ASSERT_EQ(ctr.getFunctionSup(jerkInit).cols(), 1);
}
