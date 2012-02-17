#ifndef FSM_SOLVER
#define FSM_SOLVER

////////////////////////////////////////////////////////////////////////////////
///
///\file	fsm-solver.h
///\brief	A class to compute preview support state
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author	Keith François
///\version	1.0
///\date	14/02/12
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/state-solver.h>

namespace MPCWalkgen{

	class FSMSolver:public StateSolver{
		public:
			FSMSolver(VelReference * velRef, const MPCData * generalData);
			virtual ~FSMSolver();

			virtual void setSupportState(double time, int pi, SupportState & Support);
	};

}

#endif //FSM_SOLVER
