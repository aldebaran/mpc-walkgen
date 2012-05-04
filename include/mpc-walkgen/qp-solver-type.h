#ifndef _MPC_WALKGEN_QP_SOLVER_TYPE_H_
#define _MPC_WALKGEN_QP_SOLVER_TYPE_H_

namespace MPCWalkgen
{
  enum QPSolverType {
#ifdef USE_LSSOL
    QPSOLVERTYPE_LSSOL = 0,
#endif
#ifdef USE_QPOASES
    QPSOLVERTYPE_QPOASES = 1,
#endif
  };
}

#endif // _MPC_WALKGEN_QP_SOLVER_TYPE_H_

