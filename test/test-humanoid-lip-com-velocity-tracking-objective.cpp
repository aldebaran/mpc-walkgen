////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-lip-com-velocity-tracking-objective.cpp
///\brief Test of the LIP CoM velocity tracking objective function
///\author de Gourcuff Martin
///\date 12/08/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/humanoid_feet_supervisor.h"
#include "../src/model/lip_model.h"
#include "../src/function/humanoid_lip_com_velocity_tracking_objective.h"


class HumanoidLipComVelocityTrackingTest: public ::testing::Test{};

TEST_F(HumanoidLipComVelocityTrackingTest, HessianAndGradientValue)
{
  using namespace MPCWalkgen;


  int nbSamples = 2;
  Scalar samplingPeriod = 0.2;
  Scalar stepPeriod = 0.4;

  Scalar initDSLength = 1.0;

  HumanoidFeetSupervisor feetSupervisor(nbSamples,
                                        samplingPeriod);
  feetSupervisor.setStepPeriod(stepPeriod);
  feetSupervisor.setInitialDoubleSupportLength(initDSLength);
  LIPModel lip;
  //HumanoidLipComVelocityTrackingObjective obj(lip, leftFoot, rightFoot);

}


TEST_F(HumanoidLipComVelocityTrackingTest, HessianAndGradientSize)
{

  using namespace MPCWalkgen;

  //TODO: Add footSteps

  int nbSamples = 3;
  Scalar samplingPeriod = 1.0;
  bool autoCompute = true;
  VectorX variable;
  variable.setZero(2*nbSamples);
  Scalar feedbackPeriod = 0.5;

  HumanoidFeetSupervisor feetSupervisor(nbSamples,
                                        samplingPeriod);
  feetSupervisor.updateTimeline(variable, feedbackPeriod);

  LIPModel lip(nbSamples, samplingPeriod, autoCompute);
  HumanoidLipComVelocityTrackingObjective obj(lip, feetSupervisor);

  VectorX jerkInit(2*nbSamples + 2*feetSupervisor.getNbPreviewedSteps());
  jerkInit.fill(0.0);


  ASSERT_EQ(obj.getHessian().rows(), 6);
  ASSERT_EQ(obj.getHessian().cols(), 6);
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 6);
}
