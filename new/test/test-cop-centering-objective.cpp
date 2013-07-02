#include <gtest/gtest.h>
#include "../src/model/zebulon_base_model.h"
#include "../src/model/lip_model.h"
#include "../src/function/zebulon_cop_centering_objective.h"

using namespace Eigen;
using namespace MPCWalkgen;

class CopCenteringTest: public ::testing::Test{};



TEST_F(CopCenteringTest, functionValue)
{
  LIPModel m1;
  BaseModel m2;
  CopCenteringObjective obj(m1, m2);

  VectorX jerkInit(4);
  jerkInit(0) = 1.0;
  jerkInit(1) = 1.0;
  jerkInit(2) = 1.0;
  jerkInit(3) = 1.0;

  VectorX copRef(2);
  copRef(0) = 2.0;
  copRef(1) = 2.0;
  obj.setCopRefInLocalFrame(copRef);


  VectorX baseState(4);
  baseState.fill(0.0);
  baseState(0) = -1.5;
  baseState(3) = 1.0;
  m2.setStateX(baseState);
  baseState(0) = 0.25;
  m2.setStateY(baseState);

  VectorX comState(4);
  comState.fill(0.0);
  comState(0) = -1.0;
  comState(3) = 1.0;
  m1.setStateX(comState);
  comState(0) = 0.5;
  m1.setStateY(comState);

  ASSERT_NEAR(obj.getGradient(jerkInit)(0,0), -0.1036931, EPSILON);
  ASSERT_NEAR(obj.getGradient(jerkInit)(1,0), -0.1198756, EPSILON);
  ASSERT_NEAR(obj.getGradient(jerkInit)(2,0), 0.2669894, EPSILON);
  ASSERT_NEAR(obj.getGradient(jerkInit)(3,0), 0.3086561, EPSILON);


  ASSERT_NEAR(obj.getHessian()(0,0), 0.00418996, EPSILON);
  ASSERT_NEAR(obj.getHessian()(0,1), 0.0, EPSILON);
  ASSERT_NEAR(obj.getHessian()(0,2), -0.01078831, EPSILON);
  ASSERT_NEAR(obj.getHessian()(0,3), 0.0, EPSILON);

  ASSERT_NEAR(obj.getHessian()(1,0), 0.0, EPSILON);
  ASSERT_NEAR(obj.getHessian()(1,1), 0.00418996, EPSILON);
  ASSERT_NEAR(obj.getHessian()(1,2), 0.0, EPSILON);
  ASSERT_NEAR(obj.getHessian()(1,3), -0.01078831, EPSILON);

  ASSERT_NEAR(obj.getHessian()(2,0), -0.01078831, EPSILON);
  ASSERT_NEAR(obj.getHessian()(2,1), 0.0, EPSILON);
  ASSERT_NEAR(obj.getHessian()(2,2), 0.02777778, EPSILON);
  ASSERT_NEAR(obj.getHessian()(2,3), 0.0, EPSILON);

  ASSERT_NEAR(obj.getHessian()(3,0), 0.0, EPSILON);
  ASSERT_NEAR(obj.getHessian()(3,1), -0.01078831, EPSILON);
  ASSERT_NEAR(obj.getHessian()(3,2), 0.0, EPSILON);
  ASSERT_NEAR(obj.getHessian()(3,3), 0.02777778, EPSILON);

}


TEST_F(CopCenteringTest, sizeOfvalues)
{
  int nbSamples = 3;
  LIPModel m1(nbSamples);
  BaseModel m2(nbSamples);
  CopCenteringObjective obj(m1, m2);

  VectorX jerkInit(4*nbSamples);
  jerkInit.fill(0.0);

  ASSERT_EQ(obj.getHessian().rows(), 4*nbSamples);
  ASSERT_EQ(obj.getHessian().cols(), 4*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 4*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).cols(), 1);

}
