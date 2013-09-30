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

  /*
  using namespace MPCWalkgen;

  HumanoidFootModel leftFoot, rightFoot;
  LIPModel lip;
  HumanoidLipComVelocityTrackingObjective obj(lip, leftFoot, rightFoot);

  */
  //TO BE COMPLETED
}


TEST_F(HumanoidLipComVelocityTrackingTest, HessianAndGradientSize)
{

  using namespace MPCWalkgen;

  //TODO: Add footSteps

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
  HumanoidLipComVelocityTrackingObjective obj(lip, feetSupervisor);

  VectorX jerkInit(2*nbSamples + 2*feetSupervisor.getNbPreviewedSteps());
  jerkInit.fill(0.0);


  ASSERT_EQ(obj.getHessian().rows(), 6);
  ASSERT_EQ(obj.getHessian().cols(), 6);
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 6);
}
