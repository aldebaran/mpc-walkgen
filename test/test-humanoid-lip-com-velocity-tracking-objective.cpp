////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-lip-com-velocity-tracking-objective.cpp
///\brief Test of the LIP CoM velocity tracking objective function
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include "mpc_walkgen_gtest.h"
#include <mpc-walkgen/humanoid_feet_supervisor.h>
#include <mpc-walkgen/function/humanoid_lip_com_velocity_tracking_objective.h>

using namespace MPCWalkgen;

TYPED_TEST(MpcWalkgenTest, HessianAndGradientValue)
{
  using namespace MPCWalkgen;


  int nbSamples = 2;
  TypeParam samplingPeriod = 0.2f;
  TypeParam stepPeriod = 0.4f;

  TypeParam initDSLength = 1.0f;

  HumanoidFeetSupervisor<TypeParam> feetSupervisor(nbSamples,
                                        samplingPeriod);
  feetSupervisor.setStepPeriod(stepPeriod);
  feetSupervisor.setInitialDoubleSupportLength(initDSLength);
  LIPModel<TypeParam> lip;
  //HumanoidLipComVelocityTrackingObjective obj(lip, leftFoot, rightFoot);

}


TYPED_TEST(MpcWalkgenTest, HessianAndGradientSize)
{

  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);
  //TODO: Add footSteps

  int nbSamples = 3;
  TypeParam samplingPeriod = 1.0f;
  bool autoCompute = true;
  VectorX variable;
  variable.setZero(2*nbSamples);
  TypeParam feedbackPeriod = 0.5f;

  HumanoidFeetSupervisor<TypeParam> feetSupervisor(nbSamples,
                                        samplingPeriod);
  feetSupervisor.updateTimeline(variable, feedbackPeriod);

  LIPModel<TypeParam> lip(nbSamples, samplingPeriod, autoCompute);
  lip.setFeedbackPeriod(feedbackPeriod);

  HumanoidLipComVelocityTrackingObjective<TypeParam> obj(lip, feetSupervisor);

  VectorX jerkInit(2*nbSamples + 2*feetSupervisor.getNbPreviewedSteps());
  jerkInit.fill(0.0);

  ASSERT_EQ(obj.getHessian().rows(), 6);
  ASSERT_EQ(obj.getHessian().cols(), 6);
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 6);
}
