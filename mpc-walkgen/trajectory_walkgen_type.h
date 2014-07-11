////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_TRAJECTORY_WALKGEN_TYPE_H
#define MPC_WALKGEN_TRAJECTORY_WALKGEN_TYPE_H

#include <mpc-walkgen/api.h>

namespace MPCWalkgen
{
  template <typename Scalar>
  class MPC_WALKGEN_API  TrajectoryWalkgenWeighting
  {
  public:
    TrajectoryWalkgenWeighting()
    :velocityTracking(0.)
    ,positionTracking(0.)
    ,jerkMinimization(0.)
    {}

    Scalar velocityTracking;
    Scalar positionTracking;
    Scalar jerkMinimization;
  };

  template <typename Scalar>
  class MPC_WALKGEN_API  TrajectoryWalkgenConfig
  {
  public:
    TrajectoryWalkgenConfig()
    :withMotionConstraints(false)
    {}

    bool withMotionConstraints;
  };
}

#endif
