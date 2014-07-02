////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/model/no_dynamic_model.h>
#include <cmath>
#include <mpc-walkgen/tools.h>
#include <mpc-walkgen/constant.h>
#include "../macro.h"

using namespace MPCWalkgen;

template <typename Scalar>
NoDynamicModel<Scalar>::NoDynamicModel(int nbSamples,
                                       Scalar samplingPeriod,
                                       bool autoCompute)
  :autoCompute_(autoCompute)
  ,nbSamples_(nbSamples)
  ,samplingPeriod_(samplingPeriod)
  ,velocityLimit_(1.0)
  ,accelerationLimit_(1.0)
  ,jerkLimit_(1.0)
{
  assert(samplingPeriod>0);
  assert(nbSamples>0);

  state_.setZero(3);
  if (autoCompute_)
  {
    computeDynamics();
  }
}

template <typename Scalar>
NoDynamicModel<Scalar>::NoDynamicModel()
  :autoCompute_(true)
  ,nbSamples_(1)
  ,samplingPeriod_(1.0)
  ,velocityLimit_(1.0)
  ,accelerationLimit_(1.0)
  ,jerkLimit_(1.0)
{
  state_.setZero(3);
  if (autoCompute_)
  {
    computeDynamics();
  }
}

template <typename Scalar>
NoDynamicModel<Scalar>::~NoDynamicModel(){}

template <typename Scalar>
void NoDynamicModel<Scalar>::computeDynamics()
{
  computePosDynamic();
  computeVelDynamic();
  computeAccDynamic();
  computeJerkDynamic();
}

template <typename Scalar>
void NoDynamicModel<Scalar>::computePosDynamic()
{
  Tools::ConstantJerkDynamic<Scalar>::computePosDynamic(samplingPeriod_, samplingPeriod_,
                                                        nbSamples_, posDynamic_);

}

template <typename Scalar>
void NoDynamicModel<Scalar>::computeVelDynamic()
{
  Tools::ConstantJerkDynamic<Scalar>::computeVelDynamic(samplingPeriod_, samplingPeriod_,
                                                nbSamples_, velDynamic_);
}

template <typename Scalar>
void NoDynamicModel<Scalar>::computeAccDynamic()
{
  Tools::ConstantJerkDynamic<Scalar>::computeAccDynamic(samplingPeriod_, samplingPeriod_,
                                                nbSamples_, accDynamic_);
}

template <typename Scalar>
void NoDynamicModel<Scalar>::computeJerkDynamic()
{
  Tools::ConstantJerkDynamic<Scalar>::computeJerkDynamic(nbSamples_, jerkDynamic_);
}

template <typename Scalar>
void NoDynamicModel<Scalar>::setNbSamples(int nbSamples)
{
  assert(nbSamples>0);

  nbSamples_ = nbSamples;

  if (autoCompute_)
  {
    computeDynamics();
  }
}

template <typename Scalar>
void NoDynamicModel<Scalar>::updateState(Scalar jerk, Scalar feedBackPeriod)
{
  Tools::ConstantJerkDynamic<Scalar>::updateState(jerk, feedBackPeriod, state_);
}

template <typename Scalar>
void NoDynamicModel<Scalar>::setSamplingPeriod(Scalar samplingPeriod)
{
  assert(samplingPeriod>0);

  samplingPeriod_ = samplingPeriod;

  if (autoCompute_)
  {
    computeDynamics();
  }
}

namespace MPCWalkgen
{
  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(NoDynamicModel);
}
