////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-lip-com-jerk-minimization-objective.cpp
///\brief Test of the LIP CoM jerk minimization objective function
///\author de Gourcuff Martin
///\date 12/08/13
///
////////////////////////////////////////////////////////////////////////////////


#include <gtest/gtest.h>
#include "../src/humanoid_feet_supervisor.h"
#include "../src/model/lip_model.h"
#include "../src/function/humanoid_lip_com_jerk_minimization_objective.h"


class HumanoidLipComJerkMinimizationTest: public ::testing::Test{};


TEST_F(HumanoidLipComJerkMinimizationTest, HessianAndGradientValue)
{
  //TO BE COMPLETED
}


TEST_F(HumanoidLipComJerkMinimizationTest, HessianAndGradientSize)
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
  HumanoidLipComJerkMinimizationObjective obj(lip, feetSupervisor);

  VectorX jerkInit(2*nbSamples + 2*feetSupervisor.getNbPreviewedSteps());
  jerkInit.fill(0.0);

  ASSERT_EQ(obj.getHessian().rows(), 6);
  ASSERT_EQ(obj.getHessian().cols(), 6);
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 6);
}

