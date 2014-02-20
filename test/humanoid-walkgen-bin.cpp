////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_walkgen.h
///\brief Main program for Humanoid
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/humanoid_walkgen.h>

using namespace MPCWalkgen;
typedef double Real;


int main(void)
{
  HumanoidWalkgen<Real> walkgen;

  walkgen.solve(0.02);

  return 0;
}
