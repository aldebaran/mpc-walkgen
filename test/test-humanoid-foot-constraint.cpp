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


  HumanoidFootConstraint footCtr(lip, feetSupervisor);

  //To be completed after FSM implementation

}


TEST_F(HumanoidFootConstraintTest, sizeOfConstraints)
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


  HumanoidFootConstraint footCtr(lip, feetSupervisor);
  VectorX x0;
  x0.setZero(6);

  //TODO: To be completed
  /*
  ASSERT_EQ(footCtr.getFunction(x0).rows(), 6);
  ASSERT_EQ(footCtr.getGradient(x0.rows()).rows(), 6);
  ASSERT_EQ(footCtr.getGradient(x0.rows()).cols(), 6);
  ASSERT_EQ(footCtr.getSupBounds().rows(), 0);
  ASSERT_EQ(footCtr.getInfBounds().rows(), 0);
*/
}


