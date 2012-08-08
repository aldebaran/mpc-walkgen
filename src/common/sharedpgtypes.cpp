#include <mpc-walkgen/common/sharedpgtypes.h>

namespace MPCWalkgen
{
  BodyState::BodyState(int size){
          reset(size);
  }

  void BodyState::reset(int size){
          x.setZero(size);
          y.setZero(size);
          z.setZero(size);
          yaw.setZero(size);
          pitch.setZero(size);
          roll.setZero(size);
  }

}
