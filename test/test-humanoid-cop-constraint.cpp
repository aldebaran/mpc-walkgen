////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-cop-constraint.cpp
///\brief Test of the CoP constraint function
///\author de Gourcuff Martin
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <mpc-walkgen/type.h>
#include <mpc-walkgen/constant.h>
#include <mpc-walkgen/humanoid_feet_supervisor.h>
#include <mpc-walkgen/function/humanoid_cop_constraint.h>

using namespace MPCWalkgen;
typedef float Real;

class HumanoidCopConstraintTest: public ::testing::Test
{
protected:
  typedef Type<Real>::MatrixX MatrixX;
  typedef Type<Real>::VectorX VectorX;
  typedef Type<Real>::Vector2 Vector2;
  typedef Type<Real>::vectorOfVector2 vectorOfVector2;

  virtual void SetUp()
  {
    using namespace MPCWalkgen;

    int nbSamples = 3;
    Real samplingPeriod = 1.0;
    bool autoCompute = true;
    VectorX variable;
    variable.setZero(2*nbSamples);
    Real feedbackPeriod = 0.5;

    vectorOfVector2 p(3);
    p[0] = Vector2(1.0, 1.0);
    p[1] = Vector2(-1.0, 1.0);
    p[2] = Vector2(1.0, -1.0);

    HumanoidFeetSupervisor<Real> feetSupervisor(nbSamples,
                                                samplingPeriod);
    feetSupervisor.setLeftFootCopConvexPolygon(ConvexPolygon<Real>(p));
    feetSupervisor.setRightFootCopConvexPolygon(ConvexPolygon<Real>(p));


    feetSupervisor.updateTimeline(variable, feedbackPeriod);

    LIPModel<Real> lip(nbSamples, samplingPeriod, autoCompute);
    lip.setFeedbackPeriod(feedbackPeriod);

    HumanoidCopConstraint<Real> copCtr(lip, feetSupervisor);

    VectorX x0 = VectorX::Zero(6);

    function_ = copCtr.getFunction(x0);
    gradient_ = copCtr.getGradient(x0.rows());
    supBounds_ = copCtr.getSupBounds(x0);
    infBounds_ = copCtr.getInfBounds(x0);
  }

  VectorX function_;
  MatrixX gradient_;
  VectorX supBounds_;
  VectorX infBounds_;
};

TEST_F(HumanoidCopConstraintTest, functionValue)
{
  using namespace MPCWalkgen;

  ASSERT_TRUE(function_.isZero(Constant<Real>::EPSILON));

  for (int i=0; i<3; i++)
  {
    ASSERT_NEAR(gradient_(i, i), -2, Constant<Real>::EPSILON);
    ASSERT_NEAR(gradient_(i, i + 3), -2, Constant<Real>::EPSILON);
  }

  ASSERT_TRUE(supBounds_.isConstant(1, Constant<Real>::EPSILON));

  ASSERT_TRUE(infBounds_.isConstant(-Constant<Real>::MAXIMUM_BOUND_VALUE, Constant<Real>::EPSILON));
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

