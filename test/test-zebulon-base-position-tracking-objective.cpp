////////////////////////////////////////////////////////////////////////////////
///
///\file test-zebulon-base-position-tracking-objective.cpp
///\brief Test the Zebulon base position tracking objective function
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include "mpc_walkgen_gtest.h"
#include <mpc-walkgen/model/zebulon_base_model.h>
#include <mpc-walkgen/function/zebulon_base_position_tracking_objective.h>

TYPED_TEST(MpcWalkgenTest, functionValue)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam)

  BaseModel<TypeParam> m;
  BasePositionTrackingObjective<TypeParam> obj(m);

  VectorX jerkInit(2);
  jerkInit(0) = 1.0;
  jerkInit(1) = 1.0;

  VectorX velRef(2);
  velRef(0) = 2.0;
  velRef(1) = 2.0;
  obj.setPosRefInWorldFrame(velRef);


  VectorX baseState(3);
  baseState.fill(0);
  baseState(1) = -1.5;
  m.setStateX(baseState);
  baseState(1) = 0.5;
  m.setStateY(baseState);

  ASSERT_NEAR(obj.getGradient(jerkInit)(0,0), -0.555555, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getGradient(jerkInit)(1,0), -0.222222, Constant<TypeParam>::EPSILON);


  ASSERT_NEAR(obj.getHessian()(0,0), 0.0277777, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(1,1), 0.0277777, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(0,1), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(1,0), 0.0, Constant<TypeParam>::EPSILON);

}


TYPED_TEST(MpcWalkgenTest, sizeOfvalues)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam)

  int nbSamples = 3;
  TypeParam samplingPeriod = 1.0;
  bool autoCompute = true;
  BaseModel<TypeParam> m(nbSamples, samplingPeriod, autoCompute);
  BasePositionTrackingObjective<TypeParam> obj(m);

  VectorX jerkInit(2*nbSamples);
  jerkInit.fill(0.0);

  ASSERT_EQ(obj.getHessian().rows(), 2*nbSamples);
  ASSERT_EQ(obj.getHessian().cols(), 2*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 2*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).cols(), 1);
}
