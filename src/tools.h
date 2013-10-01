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

    /// \brief Compute inverse of matrix A using LU decomposition,
    ///        and store it in matrix Ap. All values < than eps are set to zero
    void inverseLU(const MatrixX& A, MatrixX& Ap, Scalar eps);

    /// \brief Methods relative to the computation of polynomials
    inline Scalar polynomValue(const Vector4& factor, Scalar x)  {
      return factor(0)*std::pow(x, 3) + factor(1)*std::pow(x, 2) + factor(2)*x + factor(3);
    }

    inline Scalar dPolynomValue(const Vector4& factor, Scalar x) {
      return 3.0*factor(0)*std::pow(x, 2) + 2.0*factor(1)*x + factor(2);
    }

    inline Scalar ddPolynomValue(const Vector4& factor, Scalar x) {
      return 6.0*factor(0)*x + 2.0*factor(1);
    }
  }
}

#endif //MPC_WALKGEN_TOOLS_H
