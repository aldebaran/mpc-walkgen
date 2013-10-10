////////////////////////////////////////////////////////////////////////////////
///
///\file test-zebulon-base-model.cpp
///\brief Test the Zebulon base model
///\author Lafaye Jory
///\date 20/07/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/model/zebulon_base_model.h"

class ZebulonBaseModelTest: public ::testing::Test{};


void checkLinearDynamicSize(const MPCWalkgen::LinearDynamic& dyn,
                            MPCWalkgen::Scalar nbSamples)
{
  ASSERT_EQ(dyn.U.rows(), nbSamples);
  ASSERT_EQ(dyn.U.cols(), nbSamples);

  ASSERT_EQ(dyn.UT.rows(), nbSamples);
  ASSERT_EQ(dyn.UT.cols(), nbSamples);

  ASSERT_EQ(dyn.S.rows(), nbSamples);
  ASSERT_EQ(dyn.S.cols(), 3);
}


TEST_F(ZebulonBaseModelTest, sizeOfMatrices)
{
  using namespace MPCWalkgen;

  int nbSamples = 10;
  Scalar samplingPeriod = 1.0;
  bool autoCompute = true;
  BaseModel m(nbSamples, samplingPeriod, autoCompute);

  checkLinearDynamicSize(m.getBasePosLinearDynamic(), nbSamples);
  checkLinearDynamicSize(m.getBaseVelLinearDynamic(), nbSamples);
  checkLinearDynamicSize(m.getBaseAccLinearDynamic(), nbSamples);
  checkLinearDynamicSize(m.getBaseJerkLinearDynamic(), nbSamples);
}

TEST_F(ZebulonBaseModelTest, valuesOfMatrices)
{
  using namespace MPCWalkgen;

  int nbSamples = 1;
  Scalar samplingPeriod = 2.0;
  bool autoCompute = true;
  BaseModel m(nbSamples, samplingPeriod, autoCompute);

  VectorX jerk(nbSamples);
  jerk(0) = 1.0;

  VectorX init(3);
  init(0)=2.0;
  init(1)=1.5;
  init(2)=-3.0;

  const LinearDynamic& dynPos = m.getBasePosLinearDynamic();
  Scalar pos = (dynPos.S * init + dynPos.U * jerk)(0);
  ASSERT_NEAR(pos, 0.333333333, EPSILON);

  const LinearDynamic& dynVel = m.getBaseVelLinearDynamic();
  Scalar vel = (dynVel.S * init + dynVel.U * jerk)(0);
  ASSERT_NEAR(vel, -2.5, EPSILON);

  const LinearDynamic& dynAcc = m.getBaseAccLinearDynamic();
  Scalar acc = (dynAcc.S * init + dynAcc.U * jerk)(0);
  ASSERT_NEAR(acc, -1.0, EPSILON);

  const LinearDynamic& dynJerk = m.getBaseJerkLinearDynamic();
  Scalar j = (dynJerk.S * init + dynJerk.U * jerk)(0);
  ASSERT_NEAR(j, 1.0, EPSILON);
}

