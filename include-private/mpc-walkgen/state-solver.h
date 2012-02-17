#ifndef STATE_SOLVER
#define STATE_SOLVER

////////////////////////////////////////////////////////////////////////////////
///
///\file	state-solver.h
///\brief	A abstract class to regroup all state-solvers
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author	Keith François
///\version	1.0
///\date	14/02/12
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/types.h>

namespace MPCWalkgen{

	class StateSolver{
		public:
			StateSolver(VelReference * velRef, const MPCData * generalData);
			virtual ~StateSolver();

			virtual void setSupportState(double time, int pi, SupportState & Support)=0;

		protected:
			VelReference * velRef_;
			const MPCData * generalData_;

	};

}

#endif //STATE_SOLVER
