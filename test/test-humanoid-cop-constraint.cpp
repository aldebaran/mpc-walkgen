////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-cop-constraint.cpp
///\brief Test of the CoP constraint function
///\author de Gourcuff Martin
///\date 14/08/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/humanoid_feet_supervisor.h"
#include "../src/model/lip_model.h"
#include "../src/function/humanoid_cop_constraint.h"

class HumanoidCopConstraintTest: public ::testing::Test{};



TEST_F(HumanoidCopConstraintTest, functionValue)
{
  using namespace MPCWalkgen;

  int nbSamples = 3;
  Scalar samplingPeriod = 1.0;
  int nbPreviewedSteps = 0;
  bool autoCompute = true;
  HumanoidFootModel leftFoot(nbSamples, samplingPeriod, nbPreviewedSteps),
                    rightFoot(nbSamples, samplingPeriod, nbPreviewedSteps);
  HumanoidFeetSupervisor feetSupervisor(leftFoot,
                                        rightFoot,
                                        nbSamples,
                                        samplingPeriod);
  LIPModel lip(nbSamples, samplingPeriod, autoCompute);


  HumanoidCopConstraint copCtr(lip, feetSupervisor);

  //To be completed after FSM implementation
}


TEST_F(HumanoidCopConstraintTest, sizeOfValues)
{
  using namespace MPCWalkgen;

  int nbSamples = 3;
  Scalar samplingPeriod = 1.0;
  int nbPreviewedSteps = 0;
  bool autoCompute = true;
  HumanoidFootModel leftFoot(nbSamples, samplingPeriod, nbPreviewedSteps),
                    rightFoot(nbSamples, samplingPeriod, nbPreviewedSteps);
  HumanoidFeetSupervisor feetSupervisor(leftFoot,
                                        rightFoot,
                                        nbSamples,
                                        samplingPeriod);
  LIPModel lip(nbSamples, samplingPeriod, autoCompute);


  HumanoidCopConstraint copCtr(lip, feetSupervisor);
  VectorX x0;
  x0.setZero(6);

  //TODO: To be completed
  /*
  ASSERT_EQ(copCtr.getFunction(x0).rows(), 6);
  ASSERT_EQ(copCtr.getGradient(x0.rows()).rows(), 6);
  ASSERT_EQ(copCtr.getGradient(x0.rows()).cols(), 6);
  ASSERT_EQ(copCtr.getSupBounds().rows(), 0);
  ASSERT_EQ(copCtr.getInfBounds().rows(), 0);
*/
}

