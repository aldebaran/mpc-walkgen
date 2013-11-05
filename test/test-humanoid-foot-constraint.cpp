////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-foot-constraint.cpp
///\brief Test of the foot constraint function
///\author de Gourcuff Martin
///\date 14/08/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/humanoid_feet_supervisor.h"
#include "../src/model/lip_model.h"
#include "../src/function/humanoid_foot_constraint.h"

class HumanoidFootConstraintTest: public ::testing::Test{};



TEST_F(HumanoidFootConstraintTest, functionValue)
{

  //TODO: Complete with steps
  /*
  using namespace MPCWalkgen;

  int nbSamples = 3;
  Scalar samplingPeriod = 1.0;
  bool autoCompute = true;
  HumanoidFootModel leftFoot(nbSamples, samplingPeriod),
      rightFoot(nbSamples, samplingPeriod);
  HumanoidFeetSupervisor feetSupervisor(leftFoot,
                                        rightFoot,
                                        nbSamples,
                                        samplingPeriod);
  LIPModel lip(nbSamples, samplingPeriod, autoCompute);


  HumanoidFootConstraint copCtr(lip, feetSupervisor);
  VectorX x0;
  x0.setZero(6);

  ASSERT_TRUE(copCtr.getFunction(x0).isZero(EPSILON));

  for (int i=0; i<3; i++)
  {
    ASSERT_NEAR(copCtr.getGradient(x0.rows())(i, i), -2, EPSILON);
    ASSERT_NEAR(copCtr.getGradient(x0.rows())(i, i + 3), -2, EPSILON);
  }

  ASSERT_TRUE(copCtr.getSupBounds().isConstant(1, EPSILON));
  ASSERT_TRUE(copCtr.getInfBounds().isZero(EPSILON));
  */
}


TEST_F(HumanoidFootConstraintTest, sizeOfValues)
{
  using namespace MPCWalkgen;

  int nbSamples = 3;
  Scalar samplingPeriod = 1.0;
  bool autoCompute = true;

  HumanoidFeetSupervisor feetSupervisor(nbSamples,
                                        samplingPeriod);
  LIPModel lip(nbSamples, samplingPeriod, autoCompute);


  HumanoidFootConstraint copCtr(lip, feetSupervisor);
  VectorX x0 = VectorX::Zero(6);

  ASSERT_EQ(copCtr.getFunction(x0).rows(), 0);
  ASSERT_EQ(copCtr.getGradient(x0.rows()).rows(), 0);
  ASSERT_EQ(copCtr.getGradient(x0.rows()).cols(), 6);

  ASSERT_EQ(copCtr.getSupBounds(x0).rows(), 0);
  ASSERT_EQ(copCtr.getInfBounds(x0).rows(), 0);

}


