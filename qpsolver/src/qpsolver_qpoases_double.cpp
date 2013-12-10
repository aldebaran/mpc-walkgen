#include "qpsolver_qpoases.hxx"
#include <mpc-walkgen/qpsolver_qpoases_double.h>
namespace MPCWalkgen
{
  QPSolver<double> *makeQPSolverDouble(int nbVar, int nbCtr)
  {return new QPOasesSolver<double>(nbVar, nbCtr);}
}
