#ifndef COM_BODY_ZEBULON
#define COM_BODY_ZEBULON

////////////////////////////////////////////////////////////////////////////////
///
///\file	com-body.h
///\brief	A class to store CoM rigid body
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
    class CoMBody:public RigidBody{
    public:
      CoMBody(const MPCData * generalData,
              const RobotData * robotData,
              const Interpolation * interpolation);
      virtual ~CoMBody();

      virtual void interpolate(GlobalSolution & result, double currentTime, const Reference & velRef);

    protected:
      virtual void computeDynamicsMatrices(LinearDynamics & dyn,
                                           double S, double T, int N, DynamicMatrixType type);



    };
  }
}


#endif //COM_BODY_ZEBULON
