#ifndef STATE_SOLVER
#define STATE_SOLVER

////////////////////////////////////////////////////////////////////////////////
///
///\file	state-solver.h
///\brief	A abstract class to regroup all state-solvers
///\author	Herdt Andrei
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////

#include "types.h"

namespace MPCWalkgen{
  namespace Humanoid{
    class StateSolver{
    public:
      StateSolver(VelReference * velRef, const MPCData * generalData);
      ~StateSolver();

      void setSupportState(double time, int pi, const std::vector<double> &samplingTimes_vec, SupportState & Support);

    protected:
      VelReference * velRef_;
      const MPCData * generalData_;

    };
  }
}

#endif //STATE_SOLVER
