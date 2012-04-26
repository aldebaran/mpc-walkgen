#include <mpc-walkgen/sharedpgtypes.h>

namespace MPCWalkgen
{

  BodyState::BodyState(){
          reset();
  }

  void BodyState::reset(){
          x.fill(0);
          y.fill(0);
          z.fill(0);
          yaw.fill(0);
          pitch.fill(0);
          roll.fill(0);
  }

}
