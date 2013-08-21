////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-cop-constraint.cpp
///\brief Test of the CoP constraint function
///\author de Gourcuff Martin
///\date 14/08/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/model/humanoid_foot_model.h"
#include "../src/model/lip_model.h"
#include "../src/function/humanoid_cop_constraint.h"

class HumanoidCopConstraintTest: public ::testing::Test{};



TEST_F(HumanoidCopConstraintTest, functionValue)
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


  HumanoidCopConstraint copCtr(lip, leftFoot, rightFoot);

  //To be completed after FSM implementation

}


TEST_F(HumanoidCopConstraintTest, sizeOfValues)
{
  using namespace MPCWalkgen;

  int nbSamples = 3;
  Scalar samplingPeriod = 1.0;
  bool autoCompute = true;
  int nbPreviewedSteps = 2;
  LIPModel lip(nbSamples, samplingPeriod, autoCompute);
  HumanoidFootModel leftFoot(nbSamples, samplingPeriod, nbPreviewedSteps),
      rightFoot(nbSamples, samplingPeriod, nbPreviewedSteps);


  HumanoidCopConstraint copCtr(lip, leftFoot, rightFoot);

  ASSERT_EQ(copCtr.getNbConstraints(), 12);
  ASSERT_EQ(copCtr.getFunction().rows(), 12);
}

