////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-cop-centering-objective.cpp
///\brief Test of the CoP centering objective function
///\author de Gourcuff Martin
///\date 12/08/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/humanoid_feet_supervisor.h"
#include "../src/model/lip_model.h"
#include "../src/function/humanoid_cop_centering_objective.h"

class HumanoidCopCenteringTest: public ::testing::Test{};



TEST_F(HumanoidCopCenteringTest, HessianAndGradientValue)
{

  using namespace MPCWalkgen;

  int nbSamples = 3;
  Scalar samplingPeriod = 1.0;
  bool autoCompute = true;

  HumanoidFeetSupervisor feetSupervisor(nbSamples,
                                        samplingPeriod);
  LIPModel lip(nbSamples, samplingPeriod, autoCompute);
  HumanoidCopCenteringObjective obj(lip, feetSupervisor);

  ASSERT_EQ(obj.getHessian().block(0, 0, 6, 6), MatrixX::Identity(6, 6));

  VectorX jerkInit(2*nbSamples + 2*feetSupervisor.getNbPreviewedSteps());
  jerkInit.fill(1.0);

  for (int i=0; i<2*nbSamples; i++)
  {
    ASSERT_NEAR(obj.getGradient(jerkInit)(i), 1.0, EPSILON);
  }
  for (int i=0; i<2*feetSupervisor.getNbPreviewedSteps(); i++)
  {
    ASSERT_NEAR(obj.getGradient(jerkInit)(i + 2*nbSamples), 0.0, EPSILON);
  }

}


TEST_F(HumanoidCopCenteringTest, HessianAndGradientSize)
{

  using namespace MPCWalkgen;

  int nbSamples = 3;
  Scalar samplingPeriod = 1.0;
  bool autoCompute = true;

  HumanoidFeetSupervisor feetSupervisor(nbSamples,
                                        samplingPeriod);
  LIPModel lip(nbSamples, samplingPeriod, autoCompute);
  HumanoidCopCenteringObjective obj(lip, feetSupervisor);

  VectorX jerkInit(2*nbSamples + 2*feetSupervisor.getNbPreviewedSteps());
  jerkInit.fill(0.0);


  ASSERT_EQ(obj.getHessian().rows(), 2*nbSamples + 2*feetSupervisor.getNbPreviewedSteps());
  ASSERT_EQ(obj.getHessian().cols(), 2*nbSamples + 2*feetSupervisor.getNbPreviewedSteps());
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 2*nbSamples + 2*feetSupervisor.getNbPreviewedSteps());
}

