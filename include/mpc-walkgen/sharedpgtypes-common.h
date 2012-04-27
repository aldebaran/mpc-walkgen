#ifndef SHAREDPGTYPESCOMMON_H
#define SHAREDPGTYPESCOMMON_H

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
