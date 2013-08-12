////////////////////////////////////////////////////////////////////////////////
///
///\file	zebulon_type.h
///\brief	Some structures and typedefs
///\author Lafaye Jory
///\version	1.0
///\date	19/06/13
///
////////////////////////////////////////////////////////////////////////////////
#pragma once
#ifndef MPC_WALKGEN_SHARED_TYPE_H
#define MPC_WALKGEN_SHARED_TYPE_H

#include <Eigen/Core>

#include <mpc-walkgen/api.h>

namespace MPCWalkgen
{
  /// \brief  Typedefs to abstract the choice between float and double
  typedef float Scalar;
  typedef Eigen::MatrixXf MatrixX;
  typedef Eigen::Matrix3f Matrix3;
  typedef Eigen::VectorXf VectorX;
  typedef Eigen::Vector3f Vector3;

  class MPC_WALKGEN_NEW_API Weighting
  {
  public:
    Weighting();

    Scalar velocityTracking;
    Scalar positionTracking;
    Scalar copCentering;
    Scalar jerkMinimization;
    Scalar tiltMinimization;
  };

  class MPC_WALKGEN_NEW_API Config
  {
  public:
    Config();

    bool withCopConstraints;
    bool withComConstraints;
    bool withBaseMotionConstraints;
  };

}
#endif //MPC_WALKGEN_SHARED_TYPE_H
