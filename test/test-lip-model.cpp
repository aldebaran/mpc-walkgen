////////////////////////////////////////////////////////////////////////////////
///
///\file test-lip-model.cpp
///\brief Test the LIP model
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include "mpc_walkgen_gtest.h"
#include <mpc-walkgen/model/lip_model.h>

TYPED_TEST(MpcWalkgenTest, sizeOfMatrices)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);

  int nbSamples = 10;
  TypeParam samplingPeriod = 1.0;
  bool autoCompute = true;
  TypeParam feebackPeriod = 0.25;
  LIPModel<TypeParam> m(nbSamples, samplingPeriod, autoCompute);
  m.setFeedbackPeriod(feebackPeriod);

  checkLinearDynamicSize(m.getComPosLinearDynamic(), nbSamples);
  checkLinearDynamicSize(m.getComVelLinearDynamic(), nbSamples);
  checkLinearDynamicSize(m.getComAccLinearDynamic(), nbSamples);
  checkLinearDynamicSize(m.getComJerkLinearDynamic(), nbSamples);
  checkLinearDynamicSize(m.getCopXLinearDynamic(), nbSamples);
  checkLinearDynamicSize(m.getCopYLinearDynamic(), nbSamples);


  for(int i=0; i<static_cast<int>((samplingPeriod + Constant<TypeParam>::EPSILON)/feebackPeriod); ++i)
  {
    checkLinearDynamicSize(m.getComPosLinearDynamic(i), nbSamples);
    checkLinearDynamicSize(m.getComVelLinearDynamic(i), nbSamples);
    checkLinearDynamicSize(m.getComAccLinearDynamic(i), nbSamples);
    checkLinearDynamicSize(m.getComJerkLinearDynamic(i), nbSamples);
    checkLinearDynamicSize(m.getCopXLinearDynamic(i), nbSamples);
    checkLinearDynamicSize(m.getCopYLinearDynamic(i), nbSamples);
  }
}

TYPED_TEST(MpcWalkgenTest, valuesOfMatrices)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);

  int nbSamples = 1;
  TypeParam samplingPeriod = 2.0f;
  bool autoCompute = true;
  TypeParam feebackPeriod = 0.25f;
  TypeParam comHeight = 0.45f;
  Vector3 gravity(1.0, -1.0, 9.0);
  LIPModel<TypeParam> m(nbSamples, samplingPeriod, autoCompute);

  m.setSamplingPeriod(samplingPeriod);
  m.setComHeight(comHeight);
  m.setGravity(gravity);
  m.setFeedbackPeriod(feebackPeriod);

  VectorX jerk(nbSamples);
  jerk(0) = 1.0;

  VectorX init(3);
  init(0)=2.0;
  init(1)=1.5;
  init(2)=-3.0;

  const LinearDynamic<TypeParam>& dynPos = m.getComPosLinearDynamic();
  TypeParam pos = (dynPos.S * init + dynPos.K + dynPos.U * jerk)(0);
  ASSERT_NEAR(pos, 0.333333333, Constant<TypeParam>::EPSILON);

  const LinearDynamic<TypeParam>& dynVel = m.getComVelLinearDynamic();
  TypeParam vel = (dynVel.S * init + dynVel.K + dynVel.U * jerk)(0);
  ASSERT_NEAR(vel, -2.5, Constant<TypeParam>::EPSILON);

  const LinearDynamic<TypeParam>& dynAcc = m.getComAccLinearDynamic();
  TypeParam acc = (dynAcc.S * init + dynAcc.K + dynAcc.U * jerk)(0);
  ASSERT_NEAR(acc, -1.0, Constant<TypeParam>::EPSILON);

  const LinearDynamic<TypeParam>& dynJerk = m.getComJerkLinearDynamic();
  TypeParam j = (dynJerk.S * init + dynJerk.K + dynJerk.U * jerk)(0);
  ASSERT_NEAR(j, 1.0, Constant<TypeParam>::EPSILON);

  const LinearDynamic<TypeParam>& dynCopX = m.getCopXLinearDynamic();
  TypeParam copX = (dynCopX.S * init + dynCopX.K + dynCopX.U * jerk)(0);
  ASSERT_NEAR(copX, 0.333333333, Constant<TypeParam>::EPSILON);

  const LinearDynamic<TypeParam>& dynCopY = m.getCopYLinearDynamic();
  TypeParam copY = (dynCopY.S * init + dynCopY.K + dynCopY.U * jerk)(0);
  ASSERT_NEAR(copY, 0.433333333, Constant<TypeParam>::EPSILON);
}

