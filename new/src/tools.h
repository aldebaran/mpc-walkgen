////////////////////////////////////////////////////////////////////////////////
///
///\file	tools.h
///\brief	Some tools
///\author Lafaye Jory
///\version	1.0
///\date	19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_TOOLS_H
#define MPC_WALKGEN_TOOLS_H

#include "type.h"

namespace MPCWalkgen
{
namespace Tools
{

  class ConstantJerkDynamic
  {
  public:
    static void computePosDynamic(Scalar T, int N, LinearDynamic& dyn,
                                  Scalar shiftS=0.0, Scalar shiftAcc=0.0);

    static void computeVelDynamic(Scalar T, int N, LinearDynamic& dyn);

    static void computeAccDynamic(Scalar T, int N, LinearDynamic& dyn);

    static void computeJerkDynamic(int N, LinearDynamic& dyn);

    static void updateState(Scalar jerk, Scalar T, VectorX& state);
  };

}
}

#endif //MPC_WALKGEN_TOOLS_H
