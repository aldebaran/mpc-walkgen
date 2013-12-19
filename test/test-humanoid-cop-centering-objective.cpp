////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-cop-centering-objective.cpp
///\brief Test of the CoP centering objective function
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include "mpc_walkgen_gtest.h"
#include <mpc-walkgen/humanoid_feet_supervisor.h>
#include <mpc-walkgen/function/humanoid_cop_centering_objective.h>

TYPED_TEST(MpcWalkgenTest, HessianAndGradientValue)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam)

  int nbSamples = 3;
  TypeParam samplingPeriod = 1.0;
  bool autoCompute = true;

  HumanoidFeetSupervisor<TypeParam> feetSupervisor(nbSamples,
                                        samplingPeriod);
  LIPModel<TypeParam> lip(nbSamples, samplingPeriod, autoCompute);
  HumanoidCopCenteringObjective<TypeParam> obj(lip, feetSupervisor);

  ASSERT_EQ(MatrixX::Identity(6, 6), obj.getHessian().block(0, 0, 6, 6));

  VectorX jerkInit(2*nbSamples + 2*feetSupervisor.getNbPreviewedSteps());
  jerkInit.fill(1.0);

  for (int i=0; i<2*nbSamples; i++)
  {
    ASSERT_NEAR(1.0,
                obj.getGradient(jerkInit)(i),
                Constant<TypeParam>::EPSILON);
  }
  for (int i=0; i<2*feetSupervisor.getNbPreviewedSteps(); ++i)
  {
    ASSERT_NEAR(0.0,
                obj.getGradient(jerkInit)(i + 2*nbSamples),
                Constant<TypeParam>::EPSILON);
  }

}


TYPED_TEST(MpcWalkgenTest, HessianAndGradientSize)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam)

  int nbSamples = 3;
  TypeParam samplingPeriod = 1.0;
  bool autoCompute = true;

  HumanoidFeetSupervisor<TypeParam> feetSupervisor(nbSamples,
                                        samplingPeriod);
  LIPModel<TypeParam> lip(nbSamples, samplingPeriod, autoCompute);
  HumanoidCopCenteringObjective<TypeParam> obj(lip, feetSupervisor);

  VectorX jerkInit(2*nbSamples + 2*feetSupervisor.getNbPreviewedSteps());
  jerkInit.fill(0.0);

  ASSERT_EQ(2*nbSamples + 2*feetSupervisor.getNbPreviewedSteps(),
            obj.getHessian().rows());
  ASSERT_EQ(2*nbSamples + 2*feetSupervisor.getNbPreviewedSteps(),
            obj.getHessian().cols());
  ASSERT_EQ(2*nbSamples + 2*feetSupervisor.getNbPreviewedSteps(),
            obj.getGradient(jerkInit).rows());
}

