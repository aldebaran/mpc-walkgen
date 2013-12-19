#pragma once
#ifndef MPC_WALKGEN_GTEST_H
#define MPC_WALKGEN_GTEST_H

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <mpc-walkgen/model/lip_model.h>
#include <mpc-walkgen/constant.h> // often used in tests

template <typename Scalar>
class MpcWalkgenTest: public ::testing::Test
{
};
typedef ::testing::Types<float, double> MyTypes;
TYPED_TEST_CASE(MpcWalkgenTest, MyTypes);

template <typename Scalar>
void checkLinearDynamicSize(const MPCWalkgen::LinearDynamic<Scalar>& dyn,
                            int nbSamples)
{
  ASSERT_EQ(dyn.U.rows(), nbSamples);
  ASSERT_EQ(dyn.U.cols(), nbSamples);

  ASSERT_EQ(dyn.UT.rows(), nbSamples);
  ASSERT_EQ(dyn.UT.cols(), nbSamples);

  ASSERT_EQ(dyn.S.rows(), nbSamples);
  ASSERT_EQ(dyn.S.cols(), 3);
}

#endif
