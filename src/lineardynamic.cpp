////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/lineardynamic.h>
#include "macro.h"

namespace MPCWalkgen
{
  template <typename Scalar>
  void LinearDynamic<Scalar>::reset(int nbSamples,
                                    int stateVectorSize,
                                    int variableVectorSize)
  {
    U.setZero(nbSamples, variableVectorSize);
    UT.setZero(variableVectorSize, nbSamples);

    if(nbSamples==variableVectorSize)
    {
      Uinv.setZero(nbSamples, variableVectorSize);
      UTinv.setZero(variableVectorSize, nbSamples);
    }
    else
    {
      Uinv.setConstant(nbSamples,
                       variableVectorSize,
                       std::numeric_limits<Scalar>::quiet_NaN());
      UTinv.setConstant(variableVectorSize,
                        nbSamples,
                        std::numeric_limits<Scalar>::quiet_NaN());
    }

    S.setZero(nbSamples, stateVectorSize);

    K.setZero(nbSamples);
  }

  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(LinearDynamic);
}
