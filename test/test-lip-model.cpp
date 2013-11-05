////////////////////////////////////////////////////////////////////////////////
///
///\file test-lip-model.cpp
///\brief Test the LIP model
///\author Lafaye Jory
///\date 20/07/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/model/lip_model.h"


class lipModelTest: public ::testing::Test{};


void checkLinearDynamicSize(const MPCWalkgen::LinearDynamic& dyn, MPCWalkgen::Scalar nbSamples)
{
  ASSERT_EQ(dyn.U.rows(), nbSamples);
  ASSERT_EQ(dyn.U.cols(), nbSamples);

  ASSERT_EQ(dyn.UT.rows(), nbSamples);
  ASSERT_EQ(dyn.UT.cols(), nbSamples);

  ASSERT_EQ(dyn.S.rows(), nbSamples);
  ASSERT_EQ(dyn.S.cols(), 3);

  ASSERT_EQ(dyn.K.rows(), nbSamples);
}

TEST_F(lipModelTest, sizeOfMatrices)
{
  using namespace MPCWalkgen;

  int nbSamples = 10;
  Scalar samplingPeriod = 1.0;
  bool autoCompute = true;
  Scalar feebackPeriod = 0.25;
  LIPModel m(nbSamples, samplingPeriod, autoCompute);
  m.setFeedbackPeriod(feebackPeriod);

  checkLinearDynamicSize(m.getComPosLinearDynamic(), nbSamples);
  checkLinearDynamicSize(m.getComVelLinearDynamic(), nbSamples);
  checkLinearDynamicSize(m.getComAccLinearDynamic(), nbSamples);
  checkLinearDynamicSize(m.getComJerkLinearDynamic(), nbSamples);
  checkLinearDynamicSize(m.getCopXLinearDynamic(), nbSamples);
  checkLinearDynamicSize(m.getCopYLinearDynamic(), nbSamples);


  for(int i=0; i<static_cast<int>((samplingPeriod + EPSILON)/feebackPeriod); ++i)
  {
    checkLinearDynamicSize(m.getComPosLinearDynamic(i), nbSamples);
    checkLinearDynamicSize(m.getComVelLinearDynamic(i), nbSamples);
    checkLinearDynamicSize(m.getComAccLinearDynamic(i), nbSamples);
    checkLinearDynamicSize(m.getComJerkLinearDynamic(i), nbSamples);
    checkLinearDynamicSize(m.getCopXLinearDynamic(i), nbSamples);
    checkLinearDynamicSize(m.getCopYLinearDynamic(i), nbSamples);
  }
}

TEST_F(lipModelTest, valuesOfMatrices)
{
  using namespace MPCWalkgen;

  int nbSamples = 1;
  Scalar samplingPeriod = 2.0;
  bool autoCompute = true;
  Scalar feebackPeriod = 0.25;
  Scalar comHeight = 0.45;
  Vector3 gravity(1.0, -1.0, 9.0);
  LIPModel m(nbSamples, samplingPeriod, autoCompute);

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

  const LinearDynamic& dynPos = m.getComPosLinearDynamic();
  Scalar pos = (dynPos.S * init + dynPos.K + dynPos.U * jerk)(0);
  ASSERT_NEAR(pos, 0.333333333, EPSILON);

  const LinearDynamic& dynVel = m.getComVelLinearDynamic();
  Scalar vel = (dynVel.S * init + dynVel.K + dynVel.U * jerk)(0);
  ASSERT_NEAR(vel, -2.5, EPSILON);

  const LinearDynamic& dynAcc = m.getComAccLinearDynamic();
  Scalar acc = (dynAcc.S * init + dynAcc.K + dynAcc.U * jerk)(0);
  ASSERT_NEAR(acc, -1.0, EPSILON);

  const LinearDynamic& dynJerk = m.getComJerkLinearDynamic();
  Scalar j = (dynJerk.S * init + dynJerk.K + dynJerk.U * jerk)(0);
  ASSERT_NEAR(j, 1.0, EPSILON);

  const LinearDynamic& dynCopX = m.getCopXLinearDynamic();
  Scalar copX = (dynCopX.S * init + dynCopX.K + dynCopX.U * jerk)(0);
  ASSERT_NEAR(copX, 0.333333333, EPSILON);

  const LinearDynamic& dynCopY = m.getCopYLinearDynamic();
  Scalar copY = (dynCopY.S * init + dynCopY.K + dynCopY.U * jerk)(0);
  ASSERT_NEAR(copY, 0.433333333, EPSILON);
}

