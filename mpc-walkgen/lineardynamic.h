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
#ifndef MPC_WALKGEN_LINEARDYNAMIC_H
#define MPC_WALKGEN_LINEARDYNAMIC_H

#include <mpc-walkgen/api.h>
#include <mpc-walkgen/type.h>

#ifdef _MSC_VER
# pragma warning( push )
// C4251: class needs to have DLL interface
# pragma warning( disable: 4251 )
#endif

namespace MPCWalkgen
{
  /// \brief  Matrices relative to a linear dynamic of type : Y = U X + S x + K
  ///         Where X is the control variables and x the initial state
  ///         UT is U transpose.
  ///         K is a constant which does not depend on the initial state. For now it is
  ///         always a zero vector, except for dynamics related to the CoP.
  ///         If U is square, Uinv and UTinv are its inverse and transpose inverse.
  ///         inverse, respectively. If U is not square, these matrices are filled
  ///         with NaN values.
  ///         -nbSamples stands for Y's number of rows
  ///         -stateVectorSize stands for x's number of rows
  ///         -variableVectorSize stands for X's number of rows
  template <typename Scalar>
  class MPC_WALKGEN_API LinearDynamic
  {
    TEMPLATE_TYPEDEF(Scalar)

    public:
      void reset(int nbSamples,
                 int stateVectorSize,
                 int variableVectorSize);

    public:
      MatrixX U;
      MatrixX UT;
      MatrixX Uinv;
      MatrixX UTinv;
      MatrixX S;
      VectorX K;
  };
}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
