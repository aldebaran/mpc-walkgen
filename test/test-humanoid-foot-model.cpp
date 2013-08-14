////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-foot-model.cpp
///\brief Test of the humanoid foot function
///\author de Gourcuff Martin
///\date 12/08/13
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include "../src/model/humanoid_foot_model.h"

class HumanoidFootModelTest: public ::testing::Test{};

//Check selection matrices sizes
TEST_F(HumanoidFootModelTest, sizeOfSelectionMatrices)
{
  using namespace MPCWalkgen;

  int nbSamples = 10;
  Scalar samplingPeriod = 1.0;
  int nbPreviewedSteps = 5;
  HumanoidFootModel footModel(nbSamples,
                              samplingPeriod,
                              nbPreviewedSteps);

  const HumanoidFootModel::SelectionMatrix& sel = footModel.getSelectionMatrix();

  ASSERT_EQ(sel.V.rows(), nbSamples);
  ASSERT_EQ(sel.V.cols(), footModel.getNbPreviewedSteps());

  ASSERT_EQ(sel.VT.rows(), footModel.getNbPreviewedSteps());
  ASSERT_EQ(sel.VT.cols(), nbSamples);

  ASSERT_EQ(sel.V0.rows(), nbSamples);
  ASSERT_EQ(sel.V0.cols(), 1);

  ASSERT_EQ(sel.V0T.rows(), 1);
  ASSERT_EQ(sel.V0T.cols(), nbSamples);
}

//Check if all elements of the selection matrix are zeros or ones
TEST_F(HumanoidFootModelTest, contentOfSelectionMatrices)
{
  using namespace MPCWalkgen;

  int nbSamples = 10;
  Scalar samplingPeriod = 1.0;
  int nbPreviewedSteps = 5;
  float rowSum = 0;
  HumanoidFootModel footModel(nbSamples,
                              samplingPeriod,
                              nbPreviewedSteps);

  const HumanoidFootModel::SelectionMatrix& sel = footModel.getSelectionMatrix();

  for (int i=0; i<nbSamples; i++)
  {
    for (int j=0; j<footModel.getNbPreviewedSteps(); j++)
    {
      rowSum += static_cast<float>(sel.V(i,j));
    }
    ASSERT_TRUE(std::fabs(rowSum)<=EPSILON||std::fabs(rowSum-1)<=EPSILON);
  }
}

//Check foot position dynamic matrices sizes
TEST_F(HumanoidFootModelTest, sizeOfDynamicMatrices)
{
  using namespace MPCWalkgen;

  int nbSamples = 10;
  Scalar samplingPeriod = 1.0;
  int nbPreviewedSteps = 5;
  HumanoidFootModel footModel(nbSamples,
                              samplingPeriod,
                              nbPreviewedSteps);

  const LinearDynamic& dyn = footModel.getFootPosLinearDynamic();

  ASSERT_EQ(dyn.U.rows(), nbSamples);
  ASSERT_EQ(dyn.U.cols(), footModel.getNbPreviewedSteps());

  ASSERT_EQ(dyn.UT.rows(), footModel.getNbPreviewedSteps());
  ASSERT_EQ(dyn.UT.cols(), nbSamples);

  ASSERT_EQ(dyn.S.rows(), nbSamples);
  ASSERT_EQ(dyn.S.cols(), 1);

  ASSERT_EQ(dyn.ST.rows(), 1);
  ASSERT_EQ(dyn.ST.cols(), nbSamples);
}



