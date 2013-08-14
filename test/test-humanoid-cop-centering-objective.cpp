////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-cop-centering-objective.cpp
///\brief Test of the CoP centering objective function
///\author de Gourcuff Martin
///\date 12/08/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/model/humanoid_foot_model.h"
#include "../src/model/lip_model.h"
#include "../src/function/humanoid_cop_centering_objective.h"

class HumanoidCopCenteringTest: public ::testing::Test{};



TEST_F(HumanoidCopCenteringTest, functionValue)
{
  using namespace MPCWalkgen;

  int nbSamples = 3;
  Scalar samplingPeriod = 1.0;
  int nbPreviewedSteps = 2;
  bool autoCompute = true;
  HumanoidFootModel leftFoot(nbSamples, samplingPeriod, nbPreviewedSteps),
      rightFoot(nbSamples, samplingPeriod, nbPreviewedSteps);
  LIPModel lip(nbSamples, samplingPeriod, autoCompute);
  HumanoidCopCenteringObjective obj(lip, leftFoot, rightFoot);

  ASSERT_EQ(leftFoot.getNbPreviewedSteps(), rightFoot.getNbPreviewedSteps());

  ASSERT_EQ(obj.getHessian().block(0, 0, 6, 6), MatrixX::Identity(6, 6));
  ASSERT_EQ(obj.getHessian().block(6, 6, 4, 4), MatrixX::Zero(4, 4));

  VectorX jerkInit(2*nbSamples + 2*leftFoot.getNbPreviewedSteps());
  jerkInit.fill(1.0);

  for (int i=0; i<2*nbSamples; i++)
  {
    ASSERT_NEAR(obj.getGradient(jerkInit)(i), jerkInit(i), EPSILON);
  }
  for (int i=2*nbSamples; i<2*leftFoot.getNbPreviewedSteps(); i++)
  {
    ASSERT_NEAR(obj.getGradient(jerkInit)(i), 0., EPSILON);
  }
}


TEST_F(HumanoidCopCenteringTest, sizeOfValues)
{
  using namespace MPCWalkgen;

  int nbSamples = 3;
  Scalar samplingPeriod = 1.0;
  int nbPreviewedSteps = 2;
  bool autoCompute = true;
  HumanoidFootModel leftFoot(nbSamples, samplingPeriod, nbPreviewedSteps),
      rightFoot(nbSamples, samplingPeriod, nbPreviewedSteps);
  LIPModel lip(nbSamples, samplingPeriod, autoCompute);
  HumanoidCopCenteringObjective obj(lip, leftFoot, rightFoot);

  VectorX jerkInit(2*nbSamples + 2*leftFoot.getNbPreviewedSteps());
  jerkInit.fill(0.0);


  ASSERT_EQ(obj.getHessian().rows(), 2*nbSamples + 2*leftFoot.getNbPreviewedSteps());
  ASSERT_EQ(obj.getHessian().cols(), 2*nbSamples + 2*leftFoot.getNbPreviewedSteps());
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 2*nbSamples + 2*leftFoot.getNbPreviewedSteps());
  ASSERT_EQ(obj.getGradient(jerkInit).cols(), 1);

}

