////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-lip-com-jerk-minimization-objective.cpp
///\brief Test of the LIP CoM jerk minimization objective function
///\author de Gourcuff Martin
///\date 12/08/13
///
////////////////////////////////////////////////////////////////////////////////


#include <gtest/gtest.h>
#include "../src/model/humanoid_foot_model.h"
#include "../src/model/lip_model.h"
#include "../src/function/humanoid_lip_com_jerk_minimization_objective.h"


class HumanoidLipComJerkMinimizationTest: public ::testing::Test{};


TEST_F(HumanoidLipComJerkMinimizationTest, functionValue)
{
  //TO BE COMPLETED
}


TEST_F(HumanoidLipComJerkMinimizationTest, sizeOfvalues)
{
  using namespace MPCWalkgen;

  int nbSamples = 3;
  Scalar samplingPeriod = 1.0;
  int nbPreviewedSteps = 2;
  bool autoCompute = true;
  HumanoidFootModel leftFoot(nbSamples, samplingPeriod, nbPreviewedSteps),
      rightFoot(nbSamples, samplingPeriod, nbPreviewedSteps);
  LIPModel lip(nbSamples, samplingPeriod, autoCompute);
  HumanoidLipComJerkMinimizationObjective obj(lip, leftFoot, rightFoot);

  ASSERT_EQ(leftFoot.getNbPreviewedSteps(), rightFoot.getNbPreviewedSteps());

  VectorX jerkInit(2*nbSamples + 2*leftFoot.getNbPreviewedSteps());
  jerkInit.fill(0.0);

  ASSERT_EQ(obj.getHessian().rows(), 2*nbSamples + 2*leftFoot.getNbPreviewedSteps());
  ASSERT_EQ(obj.getHessian().cols(), 2*nbSamples + 2*leftFoot.getNbPreviewedSteps());
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 2*nbSamples + 2*leftFoot.getNbPreviewedSteps());
  ASSERT_EQ(obj.getGradient(jerkInit).cols(), 1);
}

