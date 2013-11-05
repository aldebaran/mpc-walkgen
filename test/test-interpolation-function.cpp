////////////////////////////////////////////////////////////////////////////////
///
///\file test-interpolation-function.cpp
///\brief Test the cubic spline interpolator methods from type.cpp
///\author de Gourcuff Martin
///\date 03/09/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/type.h"
#include "../src/tools.h"

using namespace Eigen;
using namespace MPCWalkgen;

class InterpolationTest: public ::testing::Test
{
  protected:

    virtual void SetUp()
    {
using namespace MPCWalkgen;

      T_ = 10;
      initialstate_ << 1, 2, 3;
      finalState_ << 4, 5, 6;
      factor_.resize(12);
      interpolator_.computePolynomialNormalisedFactors(factor_,
                                                      initialstate_,
                                                      finalState_,
                                                      T_);
    }

    MPCWalkgen::Interpolator interpolator_;
    Scalar T_;
    Scalar t_;
    MPCWalkgen::Vector3 initialstate_;
    MPCWalkgen::Vector3 finalState_;
    MPCWalkgen::VectorX factor_;
    MPCWalkgen::Vector4 subfactor_;
    MPCWalkgen::Vector4 subfactor1_;
    MPCWalkgen::Vector4 subfactor2_;
};


TEST_F(InterpolationTest, initialConstraints)
{
  using namespace MPCWalkgen;

  t_ = 0.0;

  interpolator_.selectFactors(subfactor_, factor_, T_/6, T_);

  ASSERT_NEAR (Tools::polynomValue(subfactor_, t_/T_), initialstate_[0], EPSILON);
  ASSERT_NEAR (Tools::dPolynomValue(subfactor_, t_/T_)/T_, initialstate_[1], EPSILON);
  ASSERT_NEAR (Tools::ddPolynomValue(subfactor_, t_/T_)/(T_*T_), initialstate_[2], EPSILON);
}


TEST_F(InterpolationTest, finalConstraints)
{
    using namespace MPCWalkgen;

  t_ = T_;

  interpolator_.selectFactors(subfactor_, factor_, 5*T_/6, T_);

  ASSERT_NEAR (Tools::polynomValue(subfactor_, t_/T_), finalState_[0], EPSILON);
  ASSERT_NEAR (Tools::dPolynomValue(subfactor_, t_/T_)/T_, finalState_[1], EPSILON);
  ASSERT_NEAR (Tools::ddPolynomValue(subfactor_, t_/T_)/(T_*T_), finalState_[2], EPSILON);
}


TEST_F(InterpolationTest, firstJunction)
{
      using namespace MPCWalkgen;

  t_ = T_/3;

  interpolator_.selectFactors(subfactor1_, factor_, T_/6, T_);
  interpolator_.selectFactors(subfactor2_, factor_, T_/2, T_);

  ASSERT_NEAR (Tools::polynomValue(subfactor1_, t_/T_), Tools::polynomValue(subfactor2_, t_/T_), EPSILON);
  ASSERT_NEAR (Tools::dPolynomValue(subfactor1_, t_/T_)/T_, Tools::dPolynomValue(subfactor2_, t_/T_)/T_, EPSILON);
  ASSERT_NEAR (Tools::ddPolynomValue(subfactor1_, t_/T_)/(T_*T_), Tools::ddPolynomValue(subfactor2_, t_/T_)/(T_*T_), EPSILON);
}


TEST_F(InterpolationTest, secondJunction)
{
      using namespace MPCWalkgen;

  t_ = 2*T_/3;

  interpolator_.selectFactors(subfactor1_, factor_, T_/2, T_);
  interpolator_.selectFactors(subfactor2_, factor_, 5*T_/6, T_);

  ASSERT_NEAR (Tools::polynomValue(subfactor1_, t_/T_), Tools::polynomValue(subfactor2_, t_/T_), EPSILON);
  ASSERT_NEAR (Tools::dPolynomValue(subfactor1_, t_/T_)/T_, Tools::dPolynomValue(subfactor2_, t_/T_)/T_, EPSILON);
  ASSERT_NEAR (Tools::ddPolynomValue(subfactor1_, t_/T_)/(T_*T_), Tools::ddPolynomValue(subfactor2_, t_/T_)/(T_*T_), EPSILON);
}


TEST_F(InterpolationTest, firstPolynomValue)
{
  t_ = T_/6;

  interpolator_.selectFactors(subfactor_, factor_, T_/6, T_);

  ASSERT_NEAR (Tools::polynomValue(subfactor_, t_/T_), 7.12731, EPSILON);
  ASSERT_NEAR (Tools::dPolynomValue(subfactor_, t_/T_), 45.2917, EPSILON);
  ASSERT_NEAR (Tools::ddPolynomValue(subfactor_, t_/T_), 3.5, EPSILON);
}


TEST_F(InterpolationTest, secondPolynomValue)
{
  t_ = T_/2;

  interpolator_.selectFactors(subfactor_, factor_, T_/2, T_);

  ASSERT_NEAR (Tools::polynomValue(subfactor_, t_/T_), 13.3333, EPSILON);
  ASSERT_NEAR (Tools::dPolynomValue(subfactor_, t_/T_), -18.25, EPSILON);
  ASSERT_NEAR (Tools::ddPolynomValue(subfactor_, t_/T_), -180, EPSILON);
}


TEST_F(InterpolationTest, thirdPolynomValue)
{
  t_ = 5*T_/6;

  interpolator_.selectFactors(subfactor_, factor_, 5*T_/6, T_);

  ASSERT_NEAR (Tools::polynomValue(subfactor_, t_/T_), 2.45602, EPSILON);
  ASSERT_NEAR (Tools::dPolynomValue(subfactor_, t_/T_), -22.2083, EPSILON);
  ASSERT_NEAR (Tools::ddPolynomValue(subfactor_, t_/T_), 266.5, EPSILON);
}

