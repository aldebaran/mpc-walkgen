#include "state-solver.h"

using namespace MPCWalkgen;

StateSolver::StateSolver(VelReference * velRef, const MPCData * generalData)
	:velRef_(velRef)
	,generalData_(generalData)
{}

StateSolver::~StateSolver(){}
