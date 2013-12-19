////////////////////////////////////////////////////////////////////////////////
///
///\file test-humanoid-feet-supervisor.cpp
///\brief Test of the humanoid foot function
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include "mpc_walkgen_gtest.h"
#include <mpc-walkgen/humanoid_feet_supervisor.h>

using namespace MPCWalkgen;

TYPED_TEST(MpcWalkgenTest, smoke)
{
  HumanoidFeetSupervisor<TypeParam> foo(10, 1.f);
}



