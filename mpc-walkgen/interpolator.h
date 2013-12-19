////////////////////////////////////////////////////////////////////////////////
///
///\file type.h
///\brief Some structures and typedefs
///\author Lafaye Jory
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_INTERPOLATOR_H
#define MPC_WALKGEN_INTERPOLATOR_H

#include <mpc-walkgen/api.h>
#include <mpc-walkgen/type.h>
#include <vector>
#include <cassert>
#include <iostream>

#ifdef _MSC_VER
# pragma warning( push )
// C4251: class needs to have DLL interface
# pragma warning( disable: 4251 )
#endif

namespace MPCWalkgen
{
  template <typename Scalar>
  class MPC_WALKGEN_API Interpolator
  {
    TEMPLATE_TYPEDEF(Scalar)
    public:
      Interpolator();
      ~Interpolator();

      /// \brief Computes normalised spline factors for each of its three polynoms
      ///        (of degree three). Constraints are at both ends of the spline,
      ///        and at the two junction between the three polynoms
      void computePolynomialNormalisedFactors(VectorX &factor,
                                              const Vector3 & initialstate,
                                              const Vector3 & finalState,
                                              Scalar T ) const;

      /// \brief select the proper 4 factors corresponding to one of the three polynoms in a
      ///        normalised spline of degree three
      void selectFactors(Vector4 &subfactor,
                         const VectorX &factor,
                         Scalar t,
                         Scalar T) const;
    private:
      /// \brief We have a cubic spline of three polynoms with 3 constraints
      ///        (position, velocity and acceleration) on both edges of the spline,
      ///        and 3 constraints (position velocity and acceleration) for each junctions
      ///        between the three polynoms. The spline is normalised, and the junction are
      ///        set at x=1/3 and x=2/3. We have then 2*3 + 2*3 = 12 equations
      ///        (one per constraint), and 12 factor (4 for each polynom) to compute.
      ///        As we have 3 constraints at x=0 for the first polynom, one can easily compute
      ///        the three first factors. The 9 left factors can be obtained from the 9
      ///        equations corresponding to the 9 left constraints. This set of 9 linear
      ///        equations can be written as a matricial equation AX = b_. AinvNorm_ is
      ///        then the inverse of matrix A.
      MatrixX aInvNorm_;
      mutable VectorX b_;
      mutable VectorX abc_;
  };
}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
