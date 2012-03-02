#ifndef COM_BODY
#define COM_BODY

////////////////////////////////////////////////////////////////////////////////
///
///\file	com-body.h
///\brief	A class to store CoM rigid body
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author	Keith François
///\version	1.0
///\date	16/02/12
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/rigid-body.h>
#include <Eigen/Dense>

namespace MPCWalkgen{

	class CoMBody:public RigidBody{
		public:
			CoMBody(const MPCData * generalData,
					const RobotData * robotData,
					const Interpolation * interpolation);
			virtual ~CoMBody();

			virtual void interpolate(MPCSolution & result, double currentTime, const VelReference & velRef);

		protected:
			virtual void computeDynamicsMatrices(LinearDynamics & dyn,
					double S, double T, int N, DynamicMatrixType type);

		private:
			void interpolateTrunkOrientation(MPCSolution & result,
					double /*currentTime*/, const VelReference & velRef);


	};

}


#endif //COM_BODY
