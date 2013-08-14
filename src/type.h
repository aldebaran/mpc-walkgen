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
  struct LinearDynamic
  {
      inline void reset(int nbSamples,
                        int stateVectorSize,
                        int variableVectorSize)
      {
        U.setZero(nbSamples, variableVectorSize);
        UT.setZero(variableVectorSize, nbSamples);

        //I
        if(nbSamples==variableVectorSize)
        {
          Uinv.setZero(nbSamples, variableVectorSize);
          UTinv.setZero(variableVectorSize, nbSamples);
        }
        else
        {
          Uinv.setConstant(nbSamples,
                           variableVectorSize,
                           std::numeric_limits<Scalar>::quiet_NaN());
          UTinv.setConstant(variableVectorSize,
                            nbSamples,
                            std::numeric_limits<Scalar>::quiet_NaN());

        }
        S.setZero(nbSamples, stateVectorSize);
        ST.setZero(stateVectorSize, nbSamples);
      }

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
  struct Hull
  {
      Hull():p(3)
      {
        p[0](0)=1.0;
        p[0](1)=-1.0;
        p[0](2)=0.0;

        p[1](0)=1.0;
        p[1](1)=1.0;
        p[1](2)=0.0;

        p[2](0)=-1.0;
        p[2](1)=0.0;
        p[2](2)=0.0;
      }

      Hull(std::vector<Vector3> pp):p(pp){}

      std::vector<Vector3> p;
  };

}

#endif //MPC_WALKGEN_TYPE_H
