#ifndef BASE_BODY_ZEBULON
#define BASE_BODY_ZEBULON

////////////////////////////////////////////////////////////////////////////////
///
///\file	base-body.h
///\brief	A class to store Base rigid body
///\author	Lafaye Jory
///\version	1.0
///\date	02/05/12
///
////////////////////////////////////////////////////////////////////////////////

#include "../types.h"
#include "../rigid-body.h"

#include <Eigen/Dense>

namespace MPCWalkgen{
  namespace Zebulon{
    class BaseBody:public RigidBody{
    public:
      BaseBody(const MPCData * generalData,
               const RobotData * robotData,
               const Interpolation * interpolation);
      virtual ~BaseBody();

      virtual void interpolate(MPCSolution & result, double currentTime, const VelReference & velRef);

    protected:
      virtual void computeDynamicsMatrices(LinearDynamics & dyn,
                                           double S, double T, int N, DynamicMatrixType type);

    };
  }
}

#endif //BASE_BODY_ZEBULON
