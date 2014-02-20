////////////////////////////////////////////////////////////////////////////////
///
///\file test-zebulon-cop-centering-objective.cpp
///\brief Test the Zebulon CoP centering objective function
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include "mpc_walkgen_gtest.h"
#include <mpc-walkgen/model/zebulon_base_model.h>
#include <mpc-walkgen/model/lip_model.h>
#include <mpc-walkgen/function/zebulon_cop_centering_objective.h>

TYPED_TEST(MpcWalkgenTest, functionValue)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);

  LIPModel<TypeParam> m1;
  BaseModel<TypeParam> m2;
  CopCenteringObjective<TypeParam> obj(m1, m2);

  VectorX jerkInit(4);
  jerkInit(0) = 1.0;
  jerkInit(1) = 1.0;
  jerkInit(2) = 1.0;
  jerkInit(3) = 1.0;

  VectorX copRef(2);
  copRef(0) = 2.0;
  copRef(1) = 2.0;
  obj.setCopRefInLocalFrame(copRef);


  VectorX baseState(3);
  baseState.fill(0.0);
  baseState(0) = -1.5;
  m2.setStateX(baseState);
  baseState(0) = 0.25;
  m2.setStateY(baseState);

  VectorX comState(3);
  comState.fill(0.0);
  comState(0) = -1.0;
  m1.setStateX(comState);
  comState(0) = 0.5;
  m1.setStateY(comState);

  ASSERT_NEAR(obj.getGradient(jerkInit)(0,0), -0.1036931, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getGradient(jerkInit)(1,0), -0.1198756, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getGradient(jerkInit)(2,0), 0.2669894, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getGradient(jerkInit)(3,0), 0.3086561, Constant<TypeParam>::EPSILON);


  ASSERT_NEAR(obj.getHessian()(0,0), 0.00418996, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(0,1), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(0,2), -0.01078831, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(0,3), 0.0, Constant<TypeParam>::EPSILON);

  ASSERT_NEAR(obj.getHessian()(1,0), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(1,1), 0.00418996, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(1,2), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(1,3), -0.01078831, Constant<TypeParam>::EPSILON);

  ASSERT_NEAR(obj.getHessian()(2,0), -0.01078831, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(2,1), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(2,2), 0.02777778, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(2,3), 0.0, Constant<TypeParam>::EPSILON);

  ASSERT_NEAR(obj.getHessian()(3,0), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(3,1), -0.01078831, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(3,2), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(3,3), 0.02777778, Constant<TypeParam>::EPSILON);

}


TYPED_TEST(MpcWalkgenTest, sizeOfvalues)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);

  int nbSamples = 3;
  TypeParam samplingPeriod = 1.0;
  bool autoCompute = true;
  LIPModel<TypeParam> m1(nbSamples, samplingPeriod, autoCompute);
  BaseModel<TypeParam> m2(nbSamples, samplingPeriod, autoCompute);
  CopCenteringObjective<TypeParam> obj(m1, m2);

  VectorX jerkInit(4*nbSamples);
  jerkInit.fill(0.0);

  ASSERT_EQ(obj.getHessian().rows(), 4*nbSamples);
  ASSERT_EQ(obj.getHessian().cols(), 4*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 4*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).cols(), 1);
}
