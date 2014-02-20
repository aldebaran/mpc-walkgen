////////////////////////////////////////////////////////////////////////////////
///
///\file zebulon_walkgen_abstract.h
///\brief Main program for Zebulon
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_ZEBULON_WALKGEN_TYPE_H
#define MPC_WALKGEN_ZEBULON_WALKGEN_TYPE_H

#include <mpc-walkgen/api.h>

namespace MPCWalkgen
{
  template <typename Scalar>
  class MPC_WALKGEN_API  ZebulonWalkgenWeighting
  {
  public:
    ZebulonWalkgenWeighting()
    :velocityTracking(0.)
    ,positionTracking(0.)
    ,copCentering(0.)
    ,comCentering(0.)
    ,jerkMinimization(0.)
    ,tiltMinimization(0.)
    ,tiltVelMinimization(0.)
    {}

    Scalar velocityTracking;
    Scalar positionTracking;
    Scalar copCentering;
    Scalar comCentering;
    Scalar jerkMinimization;
    Scalar tiltMinimization;
    Scalar tiltVelMinimization;
  };

  template <typename Scalar>
  class MPC_WALKGEN_API  ZebulonWalkgenConfig
  {
  public:
    ZebulonWalkgenConfig()
    :withCopConstraints(false)
    ,withComConstraints(false)
    ,withBaseMotionConstraints(false)
    ,withTiltMotionConstraints(false)
    {}

    bool withCopConstraints;
    bool withComConstraints;
    bool withBaseMotionConstraints;
    bool withTiltMotionConstraints;
  };
}

#endif
