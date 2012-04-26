/*! \file sharedpgtypes.h
  \brief Defines basic types for the Humanoid Walking Pattern Generator.
*/

#ifndef _SHARED_PG_TYPE
#define  _SHARED_PG_TYPE
#include <mpc-walkgen/api.h>
#include <deque>
#include <Eigen/Dense>
#include <vector>
#include <mpc-walkgen/solvers.h>

namespace MPCWalkgen
{
	//
	// Enum types
	//

	/// \name Enum types
	/// \{
	enum Axis{
		X,
		Y,
		Z,
		Yaw
	};

	/// \}

	//
	// Structures
	//

	/// \name Structures
	/// \{

	struct MPC_WALKGEN_API MPCData;

	struct MPC_WALKGEN_API RobotData;

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

	struct MPC_WALKGEN_API MPCSolution;
}

#endif /* _SHARED_PG_TYPE */
