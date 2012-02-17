#ifndef RIGID_BODY_SYSTEM
#define RIGID_BODY_SYSTEM

////////////////////////////////////////////////////////////////////////////////
///
///\file	rigid-body-system.h
///\brief	A class to store the rigid body model
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author	Keith François
///\version	1.0
///\date	05/01/12
///
////////////////////////////////////////////////////////////////////////////////


#include <mpc-walkgen/types.h>
#include <mpc-walkgen/rigid-body.h>

#include <Eigen/Dense>
#include <vector>

namespace MPCWalkgen{

	class RigidBodySystem{

		public:
			RigidBodySystem(const MPCData * generalData,
					const RobotData * robotData
					, const Interpolation * interpolation);
			~RigidBodySystem();

			void computeDynamics();

			void interpolateBodies(MPCSolution & solution, double currentTime, const VelReference & velRef);

			void updateBodyState(const MPCSolution & solution);

			void firstIterationDuration(double firstIterationDuration);

			ConvexHull convexHull(HullType type, const SupportState & prwSupport, bool computeLinearSystem=true, bool rotateHull=true) const;

			RigidBody * body(BodyType type);
			const RigidBody * body(BodyType type) const;

		private:
			void initConvexHulls() ;

		private:
			const MPCData * generalData_;
			const RobotData * robotData_;

			ConvexHull leftFootHull_;
			ConvexHull rightFootHull_;
			ConvexHull CoPLeftSSHull_;
			ConvexHull CoPRightSSHull_;
			ConvexHull CoPLeftDSHull_;
			ConvexHull CoPRightDSHull_;

			RigidBody* CoM_;
			RigidBody* leftFoot_;
			RigidBody* rightFoot_;

	};


}

/*! \fn MPCWalkgen::RigidBodySystem::RigidBodySystem(GeneralData * generalData)
* \brief Constructor
*/

/*! \fn MPCWalkgen::RigidBodySystem::computeDynamicMatrix()
* \brief Compute dynamic matrices of the rigid body system
*/

/*! \fn void MPCWalkgen::RigidBodySystem::updateBodiesState(MPCSolution & solution)
* \brief Update state of bodies
* \param solution [out] the solution
*/

/*! \fn void MPCWalkgen::RigidBodySystem::firstIterationDuration(double firstIterationDuration)
* \brief Setter to define the duration of the first iteration in the QP horizon
*/

/*! \fn DynamicMatrix & MPCWalkgen::RigidBodySystem::dynamic(DynamicMatrixType type)
* \brief Return dynamic matrices of desired type
*/

/*! \fn BodyState & MPCWalkgen::RigidBodySystem::state(BodyType type)
* \brief Return state of desired body
*/

/*! \fn ConvexHull MPCWalkgen::RigidBodySystem::convexHull(HullType type,
* const SupportState & prwSupport, bool computeLinearSystem=true, bool rotateHull=true) const
* \brief Return convex hull data of hull type
* \param prwSupport previewed support associated with the convexhull
* \param computeLinearSystem if false, linear inequalities will not be computed
*/

#endif //RIGID_BODY_SYSTEM
