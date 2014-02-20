////////////////////////////////////////////////////////////////////////////////
///
///\file zebulon_type.h
///\brief Some structures and typedefs
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////
#pragma once
#ifndef MPC_WALKGEN_CONSTANT_H
#define MPC_WALKGEN_CONSTANT_H

#include <mpc-walkgen/type.h>

namespace MPCWalkgen
{
  template <typename Scalar>
  struct Constant
  {
    TEMPLATE_TYPEDEF(Scalar)

    static const Scalar EPSILON;
    static const Scalar GRAVITY_NORM;
    static const Vector3 GRAVITY_VECTOR;
    static const Scalar MAXIMUM_BOUND_VALUE;
  };

  template <typename Scalar>
  const Scalar Constant<Scalar>::EPSILON = static_cast<Scalar>(0.0001);

  template <typename Scalar>
  const Scalar Constant<Scalar>::GRAVITY_NORM = static_cast<Scalar>(9.81);

  template <typename Scalar>
  const typename Type<Scalar>::Vector3 Constant<Scalar>::GRAVITY_VECTOR
                    = typename Type<Scalar>::Vector3(0, 0, Constant<Scalar>::GRAVITY_NORM);

  template <typename Scalar>
  const Scalar Constant<Scalar>::MAXIMUM_BOUND_VALUE = static_cast<Scalar>(10e10);
}

#endif
