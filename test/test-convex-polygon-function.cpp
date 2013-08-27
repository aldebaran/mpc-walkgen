////////////////////////////////////////////////////////////////////////////////
///
///\file test-convex-ConvexPolygon-function.cpp
///\brief Test of the convex polygons static functions defined in ConvexPolygon class
///\author de Gourcuff Martin
///\date 20/08/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/type.h"

class ConvexConvexPolygonTest: public ::testing::Test
{};


void createStandardConvexSet(std::vector<MPCWalkgen::Vector2>& p)
{
  using namespace MPCWalkgen;

  p.resize(5);

  p[0] = Vector2(1.0, 0.0);
  p[1] = Vector2(-1.0, -1.0);
  p[2] = Vector2(1.0, -1.0);
  p[3] = Vector2(1.0, 1.0);
  p[4] = Vector2(-1.0, 1.0);
}

TEST_F(ConvexConvexPolygonTest, angleBetweenVecs)
{
  using namespace MPCWalkgen;

  std::vector<Vector2> p(5);
  createStandardConvexSet(p);

  ASSERT_NEAR(ConvexPolygon::angleBetweenVecs(p[0], p[1]), -2.3561944961547852, EPSILON);
  ASSERT_NEAR(ConvexPolygon::angleBetweenVecs(p[0], p[2]), -0.78539818525314331, EPSILON);
  ASSERT_NEAR(ConvexPolygon::angleBetweenVecs(p[0], p[3]), 0.78539818525314331, EPSILON);
  ASSERT_NEAR(ConvexPolygon::angleBetweenVecs(p[0], p[4]), 2.3561944961547852, EPSILON);
}

TEST_F(ConvexConvexPolygonTest, geLowestAndLeftmostPointsIndex)
{
  using namespace MPCWalkgen;

  std::vector<Vector2> p(5);
  createStandardConvexSet(p);


  ASSERT_EQ(ConvexPolygon::getIndexOfLowestAndLeftmostVertice(p), 1);
}

TEST_F(ConvexConvexPolygonTest, getIndexOfSmallestAngleVertice)
{
  using namespace MPCWalkgen;

  std::vector<Vector2> p(5);
  createStandardConvexSet(p);

  ASSERT_EQ(ConvexPolygon::getIndexOfSmallestAngleVertice(2, p[1], p), 3);
}

TEST_F(ConvexConvexPolygonTest, getConvexConvexPolygon)
{
  using namespace MPCWalkgen;

  std::vector<Vector2> p1(9);
  p1[0] = Vector2(-4.0, 0.0);
  p1[1] = Vector2(-2.0, 2.0);
  p1[2] = Vector2(-2.0, -1.5);
  p1[3] = Vector2(0.0, 2.0);
  p1[4] = Vector2(3.0, 0.0);
  p1[5] = Vector2(2.0, 2.0);
  p1[6] = Vector2(2.0, -1.0);
  p1[7] = Vector2(1.0, 1.0);
  p1[8] = Vector2(1.0, 0.0);

  std::vector<Vector2> convexSet
      = ConvexPolygon::extractVertices(p1);
  ASSERT_EQ(convexSet.size(), 6);
  ASSERT_EQ(convexSet[0], p1[2]);
  ASSERT_EQ(convexSet[1], p1[6]);
  ASSERT_EQ(convexSet[2], p1[4]);
  ASSERT_EQ(convexSet[3], p1[5]);
  ASSERT_EQ(convexSet[4], p1[1]);
  ASSERT_EQ(convexSet[5], p1[0]);
}
