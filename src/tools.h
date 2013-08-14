////////////////////////////////////////////////////////////////////////////////
///
///\file tools.h
///\brief Some tools
///\author Lafaye Jory
///\date 19/06/13
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
        static void computeCopDynamic(Scalar T, int N, LinearDynamic& dyn,
                                      Scalar comHeight, Scalar gravityX,
                                      Scalar gravityY, Scalar mass, Scalar totalMass);

        static void computePosDynamic(Scalar T, int N, LinearDynamic& dyn);

        static void computeVelDynamic(Scalar T, int N, LinearDynamic& dyn);

        static void computeAccDynamic(Scalar T, int N, LinearDynamic& dyn);

        static void computeJerkDynamic(int N, LinearDynamic& dyn);

        static void updateState(Scalar jerk, Scalar T, VectorX& state);


    };

    void inverseLU(const MatrixX& A, MatrixX& Ap, Scalar eps);
  }
}

#endif //MPC_WALKGEN_TOOLS_H
