#ifndef FOOT_BODY
#define FOOT_BODY

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

	class FootBody:public RigidBody{
		public:
			FootBody(const MPCData * generalData,
					const RobotData * robotData,
					const Interpolation * interpolation, Foot type);
			virtual ~FootBody();

			virtual void interpolate(MPCSolution & result, double currentTime, const VelReference & velRef);

		protected:
			virtual void computeDynamicsMatrices(LinearDynamics & dyn,
					double S, double T, int N, DynamicMatrixType type);

		private:
			Eigen::VectorXd & getFootVector(MPCSolution & solution, Axis axis, unsigned derivative);

			void computeFootInterpolationByPolynomial(MPCSolution & result, Axis axis, int nbSampling,
					const Eigen::Vector3d & FootCurrentState,
					double T, const Eigen::Vector3d & nextSupportFootState);

		private:
			Foot footType_;
	};

}

#endif //FOOT_BODY
