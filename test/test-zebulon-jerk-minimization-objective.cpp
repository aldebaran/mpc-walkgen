////////////////////////////////////////////////////////////////////////////////
///
///\file test-zebulon-jerk-minimization-objective.cpp
///\brief Test the Zebulon jerk minimization objective function
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include "mpc_walkgen_gtest.h"
#include <mpc-walkgen/model/zebulon_base_model.h>
#include <mpc-walkgen/model/lip_model.h>
#include <mpc-walkgen/function/zebulon_jerk_minimization_objective.h>

TYPED_TEST(MpcWalkgenTest, functionValue)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam)

  BaseModel<TypeParam> m;
  LIPModel<TypeParam> l;
  JerkMinimizationObjective<TypeParam> obj(l, m);

  VectorX jerkInit(4);
  jerkInit.fill(1.0);

  ASSERT_EQ(obj.getGradient(jerkInit), jerkInit);
  ASSERT_EQ(obj.getHessian(), MatrixX::Identity(4, 4));
}


TYPED_TEST(MpcWalkgenTest, sizeOfvalues)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam)

  int nbSamples = 3;
  TypeParam samplingPeriod = 1.0;
  bool autoCompute = true;
  BaseModel<TypeParam> m(nbSamples, samplingPeriod, autoCompute);
  LIPModel<TypeParam> l(nbSamples, samplingPeriod, autoCompute);
  JerkMinimizationObjective<TypeParam> obj(l, m);

  VectorX jerkInit(4*nbSamples);
  jerkInit.fill(0.0);

  ASSERT_EQ(obj.getHessian().rows(), 4*nbSamples);
  ASSERT_EQ(obj.getHessian().cols(), 4*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 4*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).cols(), 1);
}
