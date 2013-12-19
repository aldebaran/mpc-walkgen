////////////////////////////////////////////////////////////////////////////////
///
///\file test-zebulon-base-model.cpp
///\brief Test the Zebulon base model
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include "mpc_walkgen_gtest.h"
#include <mpc-walkgen/model/zebulon_base_model.h>

using namespace MPCWalkgen;

TYPED_TEST(MpcWalkgenTest, sizeOfMatrices)
{
  using namespace MPCWalkgen;

  int nbSamples = 10;
  TypeParam samplingPeriod = 1.0;
  bool autoCompute = true;
  BaseModel<TypeParam> m(nbSamples, samplingPeriod, autoCompute);

  checkLinearDynamicSize<TypeParam>(m.getBasePosLinearDynamic(), nbSamples);
  checkLinearDynamicSize<TypeParam>(m.getBaseVelLinearDynamic(), nbSamples);
  checkLinearDynamicSize<TypeParam>(m.getBaseAccLinearDynamic(), nbSamples);
  checkLinearDynamicSize<TypeParam>(m.getBaseJerkLinearDynamic(), nbSamples);
}

TYPED_TEST(MpcWalkgenTest, valuesOfMatrices)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);

  int nbSamples = 1;
  TypeParam samplingPeriod = 2.0;
  bool autoCompute = true;
  BaseModel<TypeParam> m(nbSamples, samplingPeriod, autoCompute);

  VectorX jerk(nbSamples);
  jerk(0) = 1.0;

  VectorX init(3);
  init(0)=2.0;
  init(1)=1.5;
  init(2)=-3.0;

  const LinearDynamic<TypeParam>& dynPos = m.getBasePosLinearDynamic();
  TypeParam pos = (dynPos.S * init + dynPos.U * jerk)(0);
  ASSERT_NEAR(pos, 0.333333333, Constant<TypeParam>::EPSILON);

  const LinearDynamic<TypeParam>& dynVel = m.getBaseVelLinearDynamic();
  TypeParam vel = (dynVel.S * init + dynVel.U * jerk)(0);
  ASSERT_NEAR(vel, -2.5, Constant<TypeParam>::EPSILON);

  const LinearDynamic<TypeParam>& dynAcc = m.getBaseAccLinearDynamic();
  TypeParam acc = (dynAcc.S * init + dynAcc.U * jerk)(0);
  ASSERT_NEAR(acc, -1.0, Constant<TypeParam>::EPSILON);

  const LinearDynamic<TypeParam>& dynJerk = m.getBaseJerkLinearDynamic();
  TypeParam j = (dynJerk.S * init + dynJerk.U * jerk)(0);
  ASSERT_NEAR(j, 1.0, Constant<TypeParam>::EPSILON);
}

