////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-foot-model.cpp
///\brief Test of the humanoid foot function
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include "mpc_walkgen_gtest.h"
#include <mpc-walkgen/model/humanoid_foot_model.h>

using namespace MPCWalkgen;

TYPED_TEST(MpcWalkgenTest, smoke)
{
  HumanoidFootModel<TypeParam> foo(10, 1.f);
}



