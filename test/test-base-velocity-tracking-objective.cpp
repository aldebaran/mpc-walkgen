#include <gtest/gtest.h>
#include "../src/model/zebulon_base_model.h"
#include "../src/function/zebulon_base_velocity_tracking_objective.h"


using namespace Eigen;
using namespace MPCWalkgen;

class BaseVelocityTrackingTest: public ::testing::Test{};



TEST_F(BaseVelocityTrackingTest, functionValue)
{
  BaseModel m;
  BaseVelocityTrackingObjective obj(m);

  VectorX jerkInit(2);
  jerkInit(0) = 1.0;
  jerkInit(1) = 1.0;

  VectorX velRef(2);
  velRef(0) = 2.0;
  velRef(1) = 2.0;
  obj.setVelRefInWorldFrame(velRef);


  VectorX baseState(4);
  baseState.fill(0);
  baseState(1) = -1.5;
  baseState(3) = 1;
  m.setStateX(baseState);
  baseState(1) = 0.5;
  m.setStateY(baseState);

  ASSERT_NEAR(obj.getGradient(jerkInit)(0,0), -1.5, EPSILON);
  ASSERT_NEAR(obj.getGradient(jerkInit)(1,0), -0.5, EPSILON);


  ASSERT_NEAR(obj.getHessian()(0,0), 0.25, EPSILON);
  ASSERT_NEAR(obj.getHessian()(1,1), 0.25, EPSILON);
  ASSERT_NEAR(obj.getHessian()(0,1), 0.0, EPSILON);
  ASSERT_NEAR(obj.getHessian()(1,0), 0.0, EPSILON);

}


TEST_F(BaseVelocityTrackingTest, sizeOfvalues)
{
  int nbSamples = 3;
  BaseModel m(nbSamples);
  BaseVelocityTrackingObjective obj(m);

  VectorX jerkInit(2*nbSamples);
  jerkInit.fill(0.0);

  ASSERT_EQ(obj.getHessian().rows(), 2*nbSamples);
  ASSERT_EQ(obj.getHessian().cols(), 2*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 2*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).cols(), 1);

}
