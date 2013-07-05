////////////////////////////////////////////////////////////////////////////////
///
///\file	type.h
///\brief	Some structures and typedefs
///\author Lafaye Jory
///\version	1.0
///\date	19/06/13
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

  /// \brief  Matrices relative to a linear dynamic of type : Y = U X + S x
  ///         Where X is the control variables and x the initial state
  struct LinearDynamic
  {
    inline void reset(int N)
    {
      S.setZero(N,4);
      ST.setZero(4,N);
      U.setZero(N,N);
      UT.setZero(N,N);
    }

    MatrixX U;
    MatrixX UT;
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
