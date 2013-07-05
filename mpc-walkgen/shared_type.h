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

  struct MPC_WALKGEN_NEW_API Weighting
  {
    Weighting()
    :velocityTracking(0.0)
    ,positionTracking(0.0)
    ,copCentering(0.0)
    ,jerkMinimization(0.0)
    {}

    Scalar velocityTracking;
    Scalar positionTracking;
    Scalar copCentering;
    Scalar jerkMinimization;
  };

  struct MPC_WALKGEN_NEW_API Config
  {
    Config()
    :withCopConstraints(true)
    ,withComConstraints(true)
    ,withBaseMotionConstraints(true)
    {}

    bool withCopConstraints;
    bool withComConstraints;
    bool withBaseMotionConstraints;
  };

}
#endif //MPC_WALKGEN_SHARED_TYPE_H
