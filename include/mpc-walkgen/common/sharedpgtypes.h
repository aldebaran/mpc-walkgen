#pragma once
#ifndef MPC_WALKGEN_COMMON_SHAREDPGTYPES_H
#define MPC_WALKGEN_COMMON_SHAREDPGTYPES_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	api.h
///\brief	Definition of common types between each walkgen
///\author	Lafaye Jory
///\author      Keith François
///\author	Herdt Andrei
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/common/qp-solver-type.h>
#include <mpc-walkgen/common/api.h>

#include <Eigen/Dense>

namespace MPCWalkgen
{

  enum Axis{
          X,
          Y,
          Z,
          Yaw
  };

  struct MPC_WALKGEN_API BodyState{
    Eigen::VectorXd x;
    Eigen::VectorXd y;
    Eigen::VectorXd z;
    Eigen::VectorXd yaw;
    Eigen::VectorXd pitch;
    Eigen::VectorXd roll;

    BodyState(int size=3);

    void reset(int size=3);
  };
}
#endif // MPC_WALKGEN_COMMON_SHAREDPGTYPES_H
