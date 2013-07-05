#include <gtest/gtest.h>
#include "../src/model/zebulon_base_model.h"
#include "../src/model/lip_model.h"
#include "../src/function/zebulon_jerk_minimization_objective.h"


using namespace Eigen;
using namespace MPCWalkgen;

class JerkMinimizationTest: public ::testing::Test{};



TEST_F(JerkMinimizationTest, functionValue)
{
  BaseModel m;
  LIPModel l;
  JerkMinimizationObjective obj(l, m);

  VectorX jerkInit(4);
  jerkInit.fill(1.0);

  ASSERT_EQ(obj.getGradient(jerkInit), jerkInit);


  ASSERT_EQ(obj.getHessian(), MatrixX::Identity(4, 4));

}


TEST_F(JerkMinimizationTest, sizeOfvalues)
{
  int nbSamples = 3;
  BaseModel m(nbSamples);
  LIPModel l(nbSamples);
  JerkMinimizationObjective obj(l, m);

  VectorX jerkInit(4*nbSamples);
  jerkInit.fill(0.0);

  ASSERT_EQ(obj.getHessian().rows(), 4*nbSamples);
  ASSERT_EQ(obj.getHessian().cols(), 4*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 4*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).cols(), 1);

}
