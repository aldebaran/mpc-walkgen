////////////////////////////////////////////////////////////////////////////////
///
///\file type.h
///\brief Some structures and typedefs
///\author Lafaye Jory
///\date 19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_TYPE_H
#define MPC_WALKGEN_TYPE_H

#include <vector>
#include <cassert>
#include <iostream>
#include <mpc-walkgen/shared_type.h>

namespace MPCWalkgen
{
#define QPOASES_REAL_IS_FLOAT

  static const Scalar EPSILON = 0.0001;
  static const Scalar GRAVITY_NORM = 9.81;
  static const Vector3 GRAVITY_VECTOR = Vector3(0, 0, 9.81);

  /// \brief  Matrices relative to a linear dynamic of type : Y = U X + S x
  ///         Where X is the control variables and x the initial state
  ///         ST and UT are S and T transpose, respectively.
  ///         If U is square, Uinv and UTinv are its inverse and transpose
  ///         inverse, respectively. If U is not square, these matrices are filles
  ///         with NaN values.
  ///         -nbSamples stands for Y's number of rows
  ///         -stateVectorSize stands for x's number of rows
  ///         -variableVectorSize stands for X's number of rows
  class LinearDynamic
  {
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
      MatrixX ST;
  };


  struct QPMatrices
  {
      MatrixX Q;
      VectorX p;

      MatrixX A;
      MatrixX At;
      VectorX bu;
      VectorX bl;
      VectorX xl;
      VectorX xu;
  };

  /// \brief  Define a convex hull bounded by the points p
  class Hull
  {
    public:
      Hull();

      Hull(std::vector<Vector3> pp);

      inline const Vector3& getVectorMin() const
      {
        return vectorMin_;
      }
      inline const Vector3& getVectorMax() const
      {
        return vectorMax_;
      }

    private:
      void computeBoundsVectors();

    public:
      std::vector<Vector3> p;
      /// \brief Vector containing the minimum values of X, Y, and Z
      ///        coordinates of each vector of the hull
      Vector3 vectorMin_;
      /// \brief Vector containing the maximum values of X, Y, and Z
      ///        coordinates of each vector of the hull
      Vector3 vectorMax_;
  };

}

#endif //MPC_WALKGEN_TYPE_H
