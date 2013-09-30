////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-cop-constraint.cpp
///\brief Test of the CoP constraint function
///\author de Gourcuff Martin
///\date 14/08/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/humanoid_feet_supervisor.h"
#include "../src/model/lip_model.h"
#include "../src/function/humanoid_cop_constraint.h"



class HumanoidCopConstraintTest: public ::testing::Test
{
  protected:

    virtual void SetUp(){

      using namespace MPCWalkgen;

      int nbSamples = 3;
      Scalar samplingPeriod = 1.0;
      bool autoCompute = true;

      HumanoidFootModel leftFoot(nbSamples, samplingPeriod),
          rightFoot(nbSamples, samplingPeriod);

      std::vector<Vector2> p(3);
      p[0] = Vector2(1.0, 1.0);
      p[1] = Vector2(-1.0, 1.0);
      p[2] = Vector2(1.0, -1.0);
      leftFoot.setCopConvexPolygon(ConvexPolygon(p));
      rightFoot.setCopConvexPolygon(ConvexPolygon(p));

      HumanoidFeetSupervisor feetSupervisor(leftFoot,
                                            rightFoot,
                                            nbSamples,
                                            samplingPeriod);
      LIPModel lip(nbSamples, samplingPeriod, autoCompute);

      HumanoidCopConstraint copCtr(lip, feetSupervisor);

      VectorX x0 = VectorX::Zero(6);

      function_ = copCtr.getFunction(x0);
      gradient_ = copCtr.getGradient(x0.rows());
      supBounds_ = copCtr.getSupBounds(x0);
      infBounds_ = copCtr.getInfBounds(x0);
    }

    MPCWalkgen::VectorX function_;
    MPCWalkgen::MatrixX gradient_;
    MPCWalkgen::VectorX supBounds_;
    MPCWalkgen::VectorX infBounds_;
};



TEST_F(HumanoidCopConstraintTest, functionValue)
{
  using namespace MPCWalkgen;

  ASSERT_TRUE(function_.isZero(EPSILON));

  for (int i=0; i<3; i++)
  {
    ASSERT_NEAR(gradient_(i, i), -2, EPSILON);
    ASSERT_NEAR(gradient_(i, i + 3), -2, EPSILON);
  }

  ASSERT_TRUE(supBounds_.isConstant(1, EPSILON));

  ASSERT_TRUE(infBounds_.isConstant(-MAXIMUM_BOUND_VALUE, EPSILON));
}


TEST_F(HumanoidCopConstraintTest, sizeOfValues)
{
  using namespace MPCWalkgen;

  ASSERT_EQ(function_.rows(), 3);
  ASSERT_EQ(gradient_.rows(), 3);
  ASSERT_EQ(gradient_.cols(), 6);

  ASSERT_EQ(supBounds_.rows(), 6);
  ASSERT_EQ(infBounds_.rows(), 6);

}

