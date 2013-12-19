////////////////////////////////////////////////////////////////////////////////
///
///\file test-zebulon-cop-constraint.cpp
///\brief Test the Zebulon CoP constraints function
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include "mpc_walkgen_gtest.h"
#include <mpc-walkgen/model/zebulon_base_model.h>
#include <mpc-walkgen/model/lip_model.h>
#include <mpc-walkgen/function/zebulon_cop_constraint.h>

TYPED_TEST(MpcWalkgenTest, functionValue)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);

  LIPModel<TypeParam> m1;
  BaseModel<TypeParam> m2;

  TypeParam copLimitMin = 0.1f;
  TypeParam copLimitMax = 0.15f;

  vectorOfVector2 p(8);
  p[7] = Vector2(copLimitMax, 0.0);
  p[6] = Vector2(copLimitMin, -copLimitMin);
  p[5] = Vector2(0.0, -copLimitMax);
  p[4] = Vector2(-copLimitMin, -copLimitMin);
  p[3] = Vector2(-copLimitMax, 0.0);
  p[2] = Vector2(-copLimitMin, copLimitMin);
  p[1] = Vector2(0.0, copLimitMax);
  p[0] = Vector2(copLimitMin, copLimitMin);
  ConvexPolygon<TypeParam> h(p);
  m2.setCopSupportConvexPolygon(h);

  CopConstraint<TypeParam> ctr(m1, m2);
  VectorX jerkInit(4);

  jerkInit(0) = 0.0;
  jerkInit(1) = 0.0;
  jerkInit(2) = 0.0;
  jerkInit(3) = 0.0;
  ASSERT_TRUE(ctr.getFunction(jerkInit)(0)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(1)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(2)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(3)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(4)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(5)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(6)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(7)<0.0);

  jerkInit(0) = 0.0;
  jerkInit(1) = 0.0;
  jerkInit(2) = 6.0f*copLimitMax;
  jerkInit(3) = 0.0;
  ASSERT_TRUE(ctr.getFunction(jerkInit)(3)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(4)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(5)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(6)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(7)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(0)<0.0);
  ASSERT_NEAR(ctr.getFunction(jerkInit)(1), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(ctr.getFunction(jerkInit)(2), 0.0, Constant<TypeParam>::EPSILON);

  jerkInit(0) = 0.0;
  jerkInit(1) = 0.0;
  jerkInit(2) = -6.0f*copLimitMax;
  jerkInit(3) = 0.0;
  ASSERT_TRUE(ctr.getFunction(jerkInit)(3)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(4)<0.0);
  ASSERT_NEAR(ctr.getFunction(jerkInit)(5), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(ctr.getFunction(jerkInit)(6), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(7)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(0)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(1)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(2)<0.0);

  jerkInit(0) = 0.0;
  jerkInit(1) = 0.0;
  jerkInit(2) = 0.0;
  jerkInit(3) = 6.0f*copLimitMax;
  ASSERT_NEAR(ctr.getFunction(jerkInit)(3), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(ctr.getFunction(jerkInit)(4), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(5)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(6)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(7)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(0)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(1)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(2)<0.0);

  jerkInit(0) = 0.0;
  jerkInit(1) = 0.0;
  jerkInit(2) = 0.0;
  jerkInit(3) = -6.0f*copLimitMax;
  ASSERT_TRUE(ctr.getFunction(jerkInit)(3)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(4)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(5)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(6)<0.0);
  ASSERT_NEAR(ctr.getFunction(jerkInit)(7), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(ctr.getFunction(jerkInit)(0), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(1)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(2)<0.0);

}


TYPED_TEST(MpcWalkgenTest, sizeOfvalues)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);

  int nbSamples = 3;
  TypeParam samplingPeriod = 1.0f;
  bool autoCompute = true;
  LIPModel<TypeParam> m1(nbSamples, samplingPeriod, autoCompute);
  BaseModel<TypeParam> m2(nbSamples, samplingPeriod, autoCompute);


  CopConstraint<TypeParam> ctr(m1, m2);

  int M = m2.getCopSupportConvexPolygon().getNbVertices();

  VectorX jerkInit(4*nbSamples);
  jerkInit.fill(0.0);

  ASSERT_EQ(ctr.getGradient().rows(), nbSamples*M);
  ASSERT_EQ(ctr.getGradient().cols(), 4*nbSamples);
  ASSERT_EQ(ctr.getFunction(jerkInit).rows(), nbSamples*M);
  ASSERT_EQ(ctr.getFunction(jerkInit).cols(), 1);
}
