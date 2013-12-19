////////////////////////////////////////////////////////////////////////////////
///
///\file zebulon_type.h
///\brief Some structures and typedefs
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////
#pragma once
#ifndef MPC_WALKGEN_TYPE_H
#define MPC_WALKGEN_TYPE_H

#include <Eigen/Core>
#include <Eigen/StdVector>

#define TEMPLATE_TYPEDEF(S) \
  typedef typename Type<S>::MatrixX MatrixX; \
  typedef typename Type<S>::Matrix3 Matrix3; \
  typedef typename Type<S>::VectorX VectorX; \
  typedef typename Type<S>::Vector3 Vector3; \
  typedef typename Type<S>::Vector4 Vector4; \
  typedef typename Type<S>::Vector2 Vector2; \
  typedef typename Type<S>::vectorOfVector2 vectorOfVector2; \
  typedef typename Type<S>::vectorOfVector3 vectorOfVector3; \
  typedef typename Type<S>::vectorOfVector4 vectorOfVector4;


namespace MPCWalkgen
{
  /// \brief  Typedefs to abstract the choice between float and double.
  ///         The following "typedefed" matrices must contain Scalar type
  template <typename Scalar>
  struct Type
  {
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<Scalar, 2, 1> Vector2;
    typedef Eigen::Matrix<Scalar, 4, 1> Vector4;
    typedef std::vector<Vector2,
                        Eigen::aligned_allocator<Vector2> > vectorOfVector2;
    typedef std::vector<Vector3> vectorOfVector3;
    typedef std::vector<Vector4,
                        Eigen::aligned_allocator<Vector4> > vectorOfVector4;

  };
}

#endif
