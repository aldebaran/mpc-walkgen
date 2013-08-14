////////////////////////////////////////////////////////////////////////////////
///
///\file test-zebulon-jerk-minimization-objective.cpp
///\brief Test the Zebulon jerk minimization objective function
///\author Lafaye Jory
///\date 20/07/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/model/zebulon_base_model.h"
#include "../src/model/lip_model.h"
#include "../src/function/zebulon_jerk_minimization_objective.h"

class ZebulonJerkMinimizationTest: public ::testing::Test{};



TEST_F(ZebulonJerkMinimizationTest, functionValue)
{
  using namespace MPCWalkgen;

  BaseModel m;
  LIPModel l;
  JerkMinimizationObjective obj(l, m);

  VectorX jerkInit(4);
  jerkInit.fill(1.0);

  ASSERT_EQ(obj.getGradient(jerkInit), jerkInit);


  ASSERT_EQ(obj.getHessian(), MatrixX::Identity(4, 4));

}


TEST_F(ZebulonJerkMinimizationTest, sizeOfvalues)
{
  using namespace MPCWalkgen;

  int nbSamples = 3;
  Scalar samplingPeriod = 1.0;
  bool autoCompute = true;
  BaseModel m(nbSamples, samplingPeriod, autoCompute);
  LIPModel l(nbSamples, samplingPeriod, autoCompute);
  JerkMinimizationObjective obj(l, m);

  VectorX jerkInit(4*nbSamples);
  jerkInit.fill(0.0);

  ASSERT_EQ(obj.getHessian().rows(), 4*nbSamples);
  ASSERT_EQ(obj.getHessian().cols(), 4*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 4*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).cols(), 1);

}
