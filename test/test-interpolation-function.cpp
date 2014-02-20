////////////////////////////////////////////////////////////////////////////////
///
///\file test-interpolation-function.cpp
///\brief Test the cubic spline interpolator methods from type.cpp
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <mpc-walkgen/interpolator.h>
#include <mpc-walkgen/tools.h>
#include <mpc-walkgen/constant.h>

using namespace Eigen;
using namespace MPCWalkgen;

typedef double Real;

class InterpolationTest: public ::testing::Test
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:

  virtual void SetUp()
  {

    T_ = 10;
    initialstate_ << 1, 2, 3;
    finalState_ << 4, 5, 6;
    factor_.resize(12);
    interpolator_.computePolynomialNormalisedFactors(factor_,
                                                     initialstate_,
                                                     finalState_,
                                                     T_);
  }

  MPCWalkgen::Interpolator<Real> interpolator_;
  Real T_;
  Real t_;
  Type<Real>::Vector3 initialstate_;
  Type<Real>::Vector3 finalState_;
  Type<Real>::VectorX factor_;
  Type<Real>::Vector4 subfactor_;
  Type<Real>::Vector4 subfactor1_;
  Type<Real>::Vector4 subfactor2_;
};


TEST_F(InterpolationTest, initialConstraints)
{
  t_ = 0.0;

  interpolator_.selectFactors(subfactor_, factor_, T_/6, T_);
  ASSERT_NEAR (Tools::polynomValue<Real>(subfactor_, t_/T_),
                                         initialstate_[0], Constant<Real>::EPSILON);
  ASSERT_NEAR (Tools::dPolynomValue<Real>(subfactor_, t_/T_)/T_,
                                          initialstate_[1], Constant<Real>::EPSILON);
  ASSERT_NEAR (Tools::ddPolynomValue<Real>(subfactor_, t_/T_)/(T_*T_),
                                           initialstate_[2], Constant<Real>::EPSILON);
}


TEST_F(InterpolationTest, finalConstraints)
{
  t_ = T_;

  interpolator_.selectFactors(subfactor_, factor_, 5*T_/6, T_);

  ASSERT_NEAR (Tools::polynomValue<Real>(subfactor_, t_/T_),
                                         finalState_[0], Constant<Real>::EPSILON);
  ASSERT_NEAR (Tools::dPolynomValue<Real>(subfactor_, t_/T_)/T_,
                                          finalState_[1], Constant<Real>::EPSILON);
  ASSERT_NEAR (Tools::ddPolynomValue<Real>(subfactor_, t_/T_)/(T_*T_),
                                           finalState_[2], Constant<Real>::EPSILON);
}


TEST_F(InterpolationTest, firstJunction)
{
  t_ = T_/3;

  interpolator_.selectFactors(subfactor1_, factor_, T_/6, T_);
  interpolator_.selectFactors(subfactor2_, factor_, T_/2, T_);

  ASSERT_NEAR (Tools::polynomValue<Real>(subfactor1_, t_/T_),
                            Tools::polynomValue<Real>(subfactor2_, t_/T_), Constant<Real>::EPSILON);
  ASSERT_NEAR (Tools::dPolynomValue<Real>(subfactor1_, t_/T_)/T_,
                        Tools::dPolynomValue<Real>(subfactor2_, t_/T_)/T_, Constant<Real>::EPSILON);
  ASSERT_NEAR (Tools::ddPolynomValue<Real>(subfactor1_, t_/T_)/(T_*T_),
                  Tools::ddPolynomValue<Real>(subfactor2_, t_/T_)/(T_*T_), Constant<Real>::EPSILON);
}


TEST_F(InterpolationTest, secondJunction)
{
  t_ = 2*T_/3;

  interpolator_.selectFactors(subfactor1_, factor_, T_/2, T_);
  interpolator_.selectFactors(subfactor2_, factor_, 5*T_/6, T_);

  ASSERT_NEAR (Tools::polynomValue<Real>(subfactor1_, t_/T_),
                            Tools::polynomValue<Real>(subfactor2_, t_/T_), Constant<Real>::EPSILON);
  ASSERT_NEAR (Tools::dPolynomValue<Real>(subfactor1_, t_/T_)/T_,
                        Tools::dPolynomValue<Real>(subfactor2_, t_/T_)/T_, Constant<Real>::EPSILON);
  ASSERT_NEAR (Tools::ddPolynomValue<Real>(subfactor1_, t_/T_)/(T_*T_),
                  Tools::ddPolynomValue<Real>(subfactor2_, t_/T_)/(T_*T_), Constant<Real>::EPSILON);
}


TEST_F(InterpolationTest, firstPolynomValue)
{
  t_ = T_/6;

  interpolator_.selectFactors(subfactor_, factor_, T_/6, T_);

  ASSERT_NEAR (Tools::polynomValue<Real>(subfactor_, t_/T_), 7.12731, Constant<Real>::EPSILON);
  ASSERT_NEAR (Tools::dPolynomValue<Real>(subfactor_, t_/T_), 45.2917, Constant<Real>::EPSILON);
  ASSERT_NEAR (Tools::ddPolynomValue<Real>(subfactor_, t_/T_), 3.5, Constant<Real>::EPSILON);
}


TEST_F(InterpolationTest, secondPolynomValue)
{
  t_ = T_/2;

  interpolator_.selectFactors(subfactor_, factor_, T_/2, T_);

  ASSERT_NEAR (Tools::polynomValue<Real>(subfactor_, t_/T_), 13.3333, Constant<Real>::EPSILON);
  ASSERT_NEAR (Tools::dPolynomValue<Real>(subfactor_, t_/T_), -18.25, Constant<Real>::EPSILON);
  ASSERT_NEAR (Tools::ddPolynomValue<Real>(subfactor_, t_/T_), -180, Constant<Real>::EPSILON);
}


TEST_F(InterpolationTest, thirdPolynomValue)
{
  t_ = 5*T_/6;

  interpolator_.selectFactors(subfactor_, factor_, 5*T_/6, T_);

  ASSERT_NEAR (Tools::polynomValue<Real>(subfactor_, t_/T_), 2.45602, Constant<Real>::EPSILON);
  ASSERT_NEAR (Tools::dPolynomValue<Real>(subfactor_, t_/T_), -22.2083, Constant<Real>::EPSILON);
  ASSERT_NEAR (Tools::ddPolynomValue<Real>(subfactor_, t_/T_), 266.5, Constant<Real>::EPSILON);
}

