#include "qpsolver_qpoases.hxx"
#include <mpc-walkgen/qpsolver_qpoases_float.h>
namespace MPCWalkgen
{
  QPSolver<float> *makeQPSolverFloat(int nbVar, int nbCtr)
  {return new QPOasesSolver<float>(nbVar, nbCtr);}
}
