////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-lip-com-jerk-minimization-objective.cpp
///\brief Test of the LIP CoM jerk minimization objective function
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include "mpc_walkgen_gtest.h"
#include <mpc-walkgen/humanoid_feet_supervisor.h>
#include <mpc-walkgen/function/humanoid_lip_com_jerk_minimization_objective.h>

//TYPED_TEST(MpcWalkgenTest, HessianAndGradientValue)
//{
//
//}

TYPED_TEST(MpcWalkgenTest, HessianAndGradientSize)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam)
  //TODO: Add footSteps

  int nbSamples = 3;
  TypeParam samplingPeriod = 1.0;
  bool autoCompute = true;
  VectorX variable;
  variable.setZero(2*nbSamples);
  TypeParam feedbackPeriod = 0.5;

  HumanoidFeetSupervisor<TypeParam> feetSupervisor(nbSamples,
                                        samplingPeriod);
  feetSupervisor.updateTimeline(variable, feedbackPeriod);

  LIPModel<TypeParam> lip(nbSamples, samplingPeriod, autoCompute);
  lip.setFeedbackPeriod(feedbackPeriod);

  HumanoidLipComJerkMinimizationObjective<TypeParam> obj(lip, feetSupervisor);

  VectorX jerkInit(2*nbSamples + 2*feetSupervisor.getNbPreviewedSteps());
  jerkInit.fill(0.0);

  ASSERT_EQ(obj.getHessian().rows(), 6);
  ASSERT_EQ(obj.getHessian().cols(), 6);
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 6);
}

