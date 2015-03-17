////////////////////////////////////////////////////////////////////////////////
///
///\file test-zebulon-tilt-motion-constraint.cpp
///\brief Test the Zebulon tilt motion constraint function
///\author Justine Lança
///
////////////////////////////////////////////////////////////////////////////////

#include "mpc_walkgen_gtest.h"
#include <mpc-walkgen/model/lip_model.h>
#include <mpc-walkgen/model/zebulon_base_model.h>
#include <mpc-walkgen/function/zebulon_tilt_motion_constraint.h>

typedef float TypeParam;
using namespace MPCWalkgen;

TYPED_TEST(MpcWalkgenTest, initialization)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);

  LIPModel<TypeParam> lipModel;
  BaseModel<TypeParam> baseModel;
  TiltMotionConstraint<TypeParam> ctr(lipModel, baseModel);

  const unsigned int N = 10u;
  lipModel.setNbSamples(N);
  baseModel.setNbSamples(N);
  ctr.computeConstantPart();

  const unsigned int M = ctr.getNbConstraints();
  const MatrixX gradient = ctr.getGradient(); // (M, 4*N)
  ASSERT_EQ(4*N, gradient.cols());
  ASSERT_EQ(M, gradient.rows());

  VectorX X;
  X.setZero(4*N);
  VectorX function = ctr.getFunction(X); // (M)
  ASSERT_EQ(M, function.rows());

  // Check if gradient and function contain NaN values
  for (unsigned int r=0u; r < gradient.rows(); ++r)
  {
    for (unsigned int c=0u; c < gradient.cols(); ++c)
    {
      ASSERT_FALSE(gradient(r,c) != gradient(r,c));
    }
  }

  for (unsigned int r=0u; r < function.rows(); ++r)
  {
    ASSERT_FALSE(function(r) != function(r));
  }
}
