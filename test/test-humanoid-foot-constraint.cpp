////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-foot-constraint.cpp
///\brief Test of the foot constraint function
///\author de Gourcuff Martin
///\date 14/08/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/model/humanoid_foot_model.h"
#include "../src/model/lip_model.h"
#include "../src/function/humanoid_foot_constraint.h"

class HumanoidFootConstraintTest: public ::testing::Test{};



TEST_F(HumanoidFootConstraintTest, functionValue)
{
  using namespace MPCWalkgen;

  using namespace MPCWalkgen;

  int nbSamples = 3;
  Scalar samplingPeriod = 1.0;
  bool autoCompute = true;
  int nbPreviewedSteps = 2;
  LIPModel lip(nbSamples, samplingPeriod, autoCompute);
  HumanoidFootModel leftFoot(nbSamples, samplingPeriod, nbPreviewedSteps),
      rightFoot(nbSamples, samplingPeriod, nbPreviewedSteps);


  HumanoidFootConstraint humCtr(lip, leftFoot, rightFoot);

  //To be completed after FSM implementation

}


TEST_F(HumanoidFootConstraintTest, sizeOfValues)
{
  using namespace MPCWalkgen;

  int nbSamples = 3;
  Scalar samplingPeriod = 1.0;
  bool autoCompute = true;
  int nbPreviewedSteps = 2;
  LIPModel lip(nbSamples, samplingPeriod, autoCompute);
  HumanoidFootModel leftFoot(nbSamples, samplingPeriod, nbPreviewedSteps),
      rightFoot(nbSamples, samplingPeriod, nbPreviewedSteps);


  HumanoidFootConstraint humCtr(lip, leftFoot, rightFoot);
  VectorX x0;
  x0.setZero(10);

  ASSERT_EQ(humCtr.getFunction(x0).rows(), 6);
  ASSERT_EQ(humCtr.getGradient().rows(), 6);
  ASSERT_EQ(humCtr.getGradient().cols(), 10);
}


