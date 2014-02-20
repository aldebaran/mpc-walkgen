////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/qpsolverfactory.h>
#include <mpc-walkgen/qpsolver_qpoases_float.h>
#include <mpc-walkgen/qpsolver_qpoases_double.h>

namespace MPCWalkgen
{
  template <>
  QPSolver<double> *makeQPSolver<double>(int nbVar, int nbCtr)
  {return makeQPSolverDouble(nbVar, nbCtr);}

  template <>
  QPSolver<float> *makeQPSolver<float>(int nbVar, int nbCtr)
  {return makeQPSolverFloat(nbVar, nbCtr);}
}
