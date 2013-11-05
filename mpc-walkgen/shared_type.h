////////////////////////////////////////////////////////////////////////////////
///
///\file zebulon_type.h
///\brief Some structures and typedefs
///\author Lafaye Jory
///\date 19/06/13
///
////////////////////////////////////////////////////////////////////////////////
#pragma once
#ifndef MPC_WALKGEN_SHARED_TYPE_H
#define MPC_WALKGEN_SHARED_TYPE_H

#include <Eigen/Core>

#include <mpc-walkgen/api.h>

namespace MPCWalkgen
{
  /// \brief  Typedefs to abstract the choice between float and double.
  ///         The following "typedefed" matrices must contain Scalar type
  typedef float Scalar;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
  typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<Scalar, 2, 1> Vector2;
  typedef Eigen::Matrix<Scalar, 4, 1> Vector4;

  //Weighting and config will be moved inside zebulon_walkgen_abstract (as a Humanoid version
  //already exist in humanoid_walkgen_abstract)
  //It is not implemented for now to be sure to keep compatibility with naoqi
  class MPC_WALKGEN_NEW_API  Weighting
  {
    public:
      Weighting();

      Scalar velocityTracking;
      Scalar positionTracking;
      Scalar copCentering;
      Scalar comCentering;
      Scalar jerkMinimization;
      Scalar tiltMinimization;
  };

  class MPC_WALKGEN_NEW_API  Config
  {
    public:
      Config();

      bool withCopConstraints;
      bool withComConstraints;
      bool withBaseMotionConstraints;
  };

}
#endif //MPC_WALKGEN_SHARED_TYPE_H
