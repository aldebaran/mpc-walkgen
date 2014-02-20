////////////////////////////////////////////////////////////////////////////////
///
///\file tools.h
///\brief Some tools
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_TOOLS_H
#define MPC_WALKGEN_TOOLS_H

#include <mpc-walkgen/lineardynamic.h>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace MPCWalkgen
{
  namespace Tools
  {
    template <typename Scalar>
    class MPC_WALKGEN_API ConstantJerkDynamic
    {
      TEMPLATE_TYPEDEF(Scalar)
      public:
        /// \brief These functions compute linear dynamics dyn relative to the
        ///        CoP position or the LIP CoM position, velocity,
        ///        acceleration or jerk.
        ///        S is the remaining time before the next sample.
        ///        T is the sampling period.
        ///        N is the number of samples.

        static void computeCopDynamic(Scalar S, Scalar T,
                                      int N, LinearDynamic<Scalar>& dyn,
                                      Scalar comHeight, Scalar gravityX,
                                      Scalar gravityZ, Scalar mass,
                                      Scalar totalMass);

        static void computePosDynamic(Scalar S, Scalar T,
                                      int N, LinearDynamic<Scalar>& dyn);

        static void computeVelDynamic(Scalar S, Scalar T,
                                      int N, LinearDynamic<Scalar>& dyn);

        static void computeOrder2PosDynamic(Scalar S, Scalar T,
                                      int N, LinearDynamic<Scalar>& dyn);

        static void computeAccDynamic(Scalar S, Scalar T,
                                      int N, LinearDynamic<Scalar>& dyn);

        static void computeOrder2VelDynamic(Scalar S, Scalar T,
                                      int N, LinearDynamic<Scalar>& dyn);

        static void computeJerkDynamic(int N, LinearDynamic<Scalar>& dyn);

        static void updateState(Scalar jerk, Scalar T, VectorX& state);
    };

    /// \brief Compute inverse of matrix A using LU decomposition,
    ///        and store it in matrix Ap. All values < than eps are set to zero
    template <typename Scalar>
    void inverseLU(const typename Type<Scalar>::MatrixX& A,
                   typename Type<Scalar>::MatrixX& Ap, Scalar eps) {
      Eigen::FullPivLU<typename Type<Scalar>::MatrixX> lu(A);
      Ap = lu.inverse();
      for(int i=0; i<Ap.rows(); ++i){
        for(int j=0; j<Ap.cols(); ++j){
          if (std::abs(Ap(i,j))<eps){
            Ap(i, j) = static_cast<Scalar>(0);
          }
        }
      }
    }

    /// \brief Compute inverse of matrix A using SVD decomposition,
    ///        and store it in matrix Ap. All values < than eps are set to zero
    template <typename Scalar>
    void inverseSVD(const typename Type<Scalar>::MatrixX& A,
                    typename Type<Scalar>::MatrixX& Ap, Scalar eps=1e-15f) {
      Eigen::JacobiSVD<typename Type<Scalar>::MatrixX> svd =
          A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

      Scalar tolerance = eps * std::max(A.cols(), A.rows())
          * svd.singularValues().array().abs().maxCoeff();

      Ap = svd.matrixV()
          * ( (svd.singularValues().array().abs() > tolerance).select(
                svd.singularValues().array().inverse(), 0)
            ).matrix().asDiagonal() * svd.matrixU().adjoint();
    }

    /// \brief Methods relative to the computation of polynomials
    template <typename Scalar>
    inline Scalar polynomValue(const typename Type<Scalar>::Vector4& factor,
                               Scalar x)  {
      return factor(0)*std::pow(x, 3) +
             factor(1)*std::pow(x, 2) +
             factor(2)*x + factor(3);
    }

    template <typename Scalar>
    inline Scalar dPolynomValue(const typename Type<Scalar>::Vector4& factor,
                                Scalar x) {
      return 3.0f*factor(0)*std::pow(x, 2) + 2.0f*factor(1)*x + factor(2);
    }

    template <typename Scalar>
    inline Scalar ddPolynomValue(const typename Type<Scalar>::Vector4& factor,
                                 Scalar x) {
      return 6.0f*factor(0)*x + 2.0f*factor(1);
    }
  }
}

#endif
