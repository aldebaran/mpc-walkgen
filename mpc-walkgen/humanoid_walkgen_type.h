////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_walkgen_abstract.h
///\brief Main program for Humanoid
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_HUMANOID_WALKGEN_TYPE_H
#define MPC_WALKGEN_HUMANOID_WALKGEN_TYPE_H

#include <mpc-walkgen/api.h>
#include <mpc-walkgen/type.h>

namespace MPCWalkgen
{
  template <typename Scalar>
  class MPC_WALKGEN_API HumanoidWalkgenWeighting
  {
  public:
    HumanoidWalkgenWeighting()
    :velocityTracking(0.)
    ,copCentering(0.)
    ,jerkMinimization(0.)
    {}

    Scalar velocityTracking;
    Scalar copCentering;
    Scalar jerkMinimization;
  };

  template <typename Scalar>
  class MPC_WALKGEN_API HumanoidWalkgenConfig
  {
  public:
    HumanoidWalkgenConfig()
    :withCopConstraints(false)
    ,withFeetConstraints(false)
    {}

    bool withCopConstraints;
    bool withFeetConstraints;
  };
}

#endif
