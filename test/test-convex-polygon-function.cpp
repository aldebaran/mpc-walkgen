////////////////////////////////////////////////////////////////////////////////
///
///\file test-convex-ConvexPolygon-function.cpp
///\brief Test of the convex polygons static functions defined in ConvexPolygon
///       class
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include "mpc_walkgen_gtest.h"
#include <mpc-walkgen/type.h>
#include <mpc-walkgen/convexpolygon.h>

using namespace MPCWalkgen;

template <typename Scalar>
void createStandardConvexSet(typename Type<Scalar>::vectorOfVector2& p)
{
  using namespace MPCWalkgen;
  typedef typename Type<Scalar>::Vector2 Vector2;
  p.resize(5);

  p[0] = Vector2(1.0, 0.0);
  p[1] = Vector2(-1.0, -1.0);
  p[2] = Vector2(1.0, -1.0);
  p[3] = Vector2(1.0, 1.0);
  p[4] = Vector2(-1.0, 1.0);
}

TYPED_TEST(MpcWalkgenTest, angleBetweenVecs)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);

  vectorOfVector2 p(5);
  createStandardConvexSet<TypeParam>(p);

  ASSERT_NEAR(ConvexPolygon<TypeParam>::angleBetweenVecs(p[0], p[1]),
              -2.3561944961547852, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(ConvexPolygon<TypeParam>::angleBetweenVecs(p[0], p[2]),
              -0.78539818525314331, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(ConvexPolygon<TypeParam>::angleBetweenVecs(p[0], p[3]),
              0.78539818525314331, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(ConvexPolygon<TypeParam>::angleBetweenVecs(p[0], p[4]),
              2.3561944961547852, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(ConvexPolygon<TypeParam>::angleBetweenVecs(p[2],
                                                         Vector2(0.0, 0.0)),
              0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(ConvexPolygon<TypeParam>::angleBetweenVecs(p[0], p[0]),
              0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(ConvexPolygon<TypeParam>::angleBetweenVecs(p[0],
                                                         Vector2(2.0, 0.0)),
              0.0, Constant<TypeParam>::EPSILON);
}

TYPED_TEST(MpcWalkgenTest, geLowestAndLeftmostPointsIndex)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);

  vectorOfVector2 p(5);
  createStandardConvexSet<TypeParam>(p);

  ASSERT_EQ(1, ConvexPolygon<TypeParam>::getIndexOfLowestAndLeftmostVertice(p));

  p[2] = p[1];

  ASSERT_EQ(1, ConvexPolygon<TypeParam>::getIndexOfLowestAndLeftmostVertice(p));
}

// disabled because it fails on 32bit archs when Scalar == double
TYPED_TEST(MpcWalkgenTest, DISABLED_getIndexOfSmallestAngleVertice)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);
  vectorOfVector2 p(5);
  createStandardConvexSet<TypeParam>(p);

  ASSERT_EQ(3, ConvexPolygon<TypeParam>::getIndexOfSmallestAngleVertice(2, p[1], p));

  p[4] = p[3];

  ASSERT_EQ(3, ConvexPolygon<TypeParam>::getIndexOfSmallestAngleVertice(2, p[1], p));
}

// disabled because it fails on 32bit archs when Scalar == double
TYPED_TEST(MpcWalkgenTest, DISABLED_getConvexPolygon)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);

  vectorOfVector2 p1(9);
  p1[0] = Vector2(-4.0, 0.0);
  p1[1] = Vector2(-2.0, 2.0);
  p1[2] = Vector2(-2.0, -1.5);
  p1[3] = Vector2(0.0, 2.0);
  p1[4] = Vector2(3.0, 0.0);
  p1[5] = Vector2(2.0, 2.0);
  p1[6] = Vector2(2.0, -1.0);
  p1[7] = Vector2(1.0, 1.0);
  p1[8] = Vector2(1.0, 0.0);

  vectorOfVector2 convexSet
      = ConvexPolygon<TypeParam>::extractVertices(p1);
  ASSERT_EQ(6u, convexSet.size());
  ASSERT_TRUE(convexSet[0].isApprox(p1[2]));
  ASSERT_TRUE(convexSet[1].isApprox(p1[6]));
  ASSERT_TRUE(convexSet[2].isApprox(p1[4]));
  ASSERT_TRUE(convexSet[3].isApprox(p1[5]));
  ASSERT_TRUE(convexSet[4].isApprox(p1[1]));
  ASSERT_TRUE(convexSet[5].isApprox(p1[0]));

  p1[0] = Vector2(1.0, 0.0);
  p1[1] = Vector2(-1.0, 1.0);
  p1[2] = Vector2(-1.0, 1.0);
  p1[3] = Vector2(0.0, 0.0);
  p1[4] = Vector2(1.0, 1.0);
  p1[5] = Vector2(1.0, -1.0);
  p1[6] = Vector2(0.0, 1.0);
  p1[7] = Vector2(1.0, -1.0);
  p1[8] = Vector2(1.0, 1.0);

  convexSet = ConvexPolygon<TypeParam>::extractVertices(p1);
  ASSERT_EQ(convexSet.size(), static_cast<size_t>(3));
  ASSERT_TRUE(convexSet[0].isApprox(p1[5]));
  ASSERT_TRUE(convexSet[1].isApprox(p1[4]));
  ASSERT_TRUE(convexSet[2].isApprox(p1[1]));
}
