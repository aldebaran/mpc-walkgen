#ifndef SHAREDPGTYPESCOMMON_H
#define SHAREDPGTYPESCOMMON_H

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

#include <mpc-walkgen/solvers.h>
#include <mpc-walkgen/api.h>

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
    Eigen::Vector3d x;
    Eigen::Vector3d y;
    Eigen::Vector3d z;
    Eigen::Vector3d yaw;
    Eigen::Vector3d pitch;
    Eigen::Vector3d roll;

    BodyState();

    void reset();
  };
}
#endif // SHAREDPGTYPESCOMMON_H
