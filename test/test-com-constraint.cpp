#include <gtest/gtest.h>
#include "../src/model/zebulon_base_model.h"
#include "../src/model/lip_model.h"
#include "../src/function/zebulon_com_constraint.h"

using namespace Eigen;
using namespace MPCWalkgen;

class ComConstraintTest: public ::testing::Test{};



TEST_F(ComConstraintTest, functionValue)
{
  LIPModel m1;
  BaseModel m2;

  Scalar copLimitMin = 0.1;
  Scalar copLimitMax = 0.15;

  std::vector<Vector3> p(8);
  p[7] = Vector3(copLimitMax, 0.0, 0.0);
  p[6] = Vector3(copLimitMin, -copLimitMin, 0.0);
  p[5] = Vector3(0.0, -copLimitMax, 0.0);
  p[4] = Vector3(-copLimitMin, -copLimitMin, 0.0);
  p[3] = Vector3(-copLimitMax, 0.0, 0.0);
  p[2] = Vector3(-copLimitMin, copLimitMin, 0.0);
  p[1] = Vector3(0.0, copLimitMax, 0.0);
  p[0] = Vector3(copLimitMin, copLimitMin, 0.0);
  Hull h(p);
  m2.setComSupportHull(h);

  ComConstraint ctr(m1, m2);
  VectorX jerkInit(4);

  jerkInit(0) = 0.0;
  jerkInit(1) = 0.0;
  jerkInit(2) = 0.0;
  jerkInit(3) = 0.0;
  ASSERT_TRUE(ctr.getFunction(jerkInit)(0)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(1)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(2)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(3)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(4)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(5)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(6)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(7)<0.0);

  jerkInit(0) = 0.0;
  jerkInit(1) = 0.0;
  jerkInit(2) = 6.0*copLimitMax;
  jerkInit(3) = 0.0;
  ASSERT_TRUE(ctr.getFunction(jerkInit)(0)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(1)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(2)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(3)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(4)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(5)<0.0);
  ASSERT_NEAR(ctr.getFunction(jerkInit)(6), 0.0, EPSILON);
  ASSERT_NEAR(ctr.getFunction(jerkInit)(7), 0.0, EPSILON);

  jerkInit(0) = 0.0;
  jerkInit(1) = 0.0;
  jerkInit(2) = -6.0*copLimitMax;
  jerkInit(3) = 0.0;
  ASSERT_TRUE(ctr.getFunction(jerkInit)(0)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(1)<0.0);
  ASSERT_NEAR(ctr.getFunction(jerkInit)(2), 0.0, EPSILON);
  ASSERT_NEAR(ctr.getFunction(jerkInit)(3), 0.0, EPSILON);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(4)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(5)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(6)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(7)<0.0);

  jerkInit(0) = 0.0;
  jerkInit(1) = 0.0;
  jerkInit(2) = 0.0;
  jerkInit(3) = 6.0*copLimitMax;
  ASSERT_NEAR(ctr.getFunction(jerkInit)(0), 0.0, EPSILON);
  ASSERT_NEAR(ctr.getFunction(jerkInit)(1), 0.0, EPSILON);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(2)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(3)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(4)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(5)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(6)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(7)<0.0);


  jerkInit(0) = 0.0;
  jerkInit(1) = 0.0;
  jerkInit(2) = 0.0;
  jerkInit(3) = -6.0*copLimitMax;
  ASSERT_TRUE(ctr.getFunction(jerkInit)(0)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(1)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(2)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(3)<0.0);
  ASSERT_NEAR(ctr.getFunction(jerkInit)(4), 0.0, EPSILON);
  ASSERT_NEAR(ctr.getFunction(jerkInit)(5), 0.0, EPSILON);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(6)<0.0);
  ASSERT_TRUE(ctr.getFunction(jerkInit)(7)<0.0);

}


TEST_F(ComConstraintTest, sizeOfvalues)
{
  int nbSamples = 3;
  LIPModel m1(nbSamples);
  BaseModel m2(nbSamples);

  ComConstraint ctr(m1, m2);

  int M = m2.getComSupportHull().p.size();

  VectorX jerkInit(4*nbSamples);
  jerkInit.fill(0.0);

  ASSERT_EQ(ctr.getGradient().rows(), nbSamples*M);
  ASSERT_EQ(ctr.getGradient().cols(), 4*nbSamples);
  ASSERT_EQ(ctr.getFunction(jerkInit).rows(), nbSamples*M);
  ASSERT_EQ(ctr.getFunction(jerkInit).cols(), 1);

}
