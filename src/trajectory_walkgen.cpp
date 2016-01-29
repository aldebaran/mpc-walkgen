////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/trajectory_walkgen.h>
#include <iostream>
#include <mpc-walkgen/constant.h>
#include <cmath>
#include "macro.h"

namespace MPCWalkgen
{

template <typename Scalar>
TrajectoryWalkgen<Scalar>::TrajectoryWalkgen()
:qpoasesSolver_(makeQPSolver<Scalar>(1, 1))
,jerkMinObj_(noDynModel_)
,velTrackingObj_(noDynModel_)
,posTrackingObj_(noDynModel_)
,motionConstraint_(noDynModel_)
{
  dX_.setZero(noDynModel_.getNbSamples());
  X_.setZero(noDynModel_.getNbSamples());

  computeConstantPart();
}

template <typename Scalar>
TrajectoryWalkgen<Scalar>::~TrajectoryWalkgen(){}

template <typename Scalar>
void TrajectoryWalkgen<Scalar>::setNbSamples(int nbSamples)
{
  assert(nbSamples>0);

  noDynModel_.setNbSamples(nbSamples);

  dX_.setZero(nbSamples);
  X_.setZero(nbSamples);

  jerkMinObj_.computeConstantPart();
  velTrackingObj_.computeConstantPart();
  posTrackingObj_.computeConstantPart();
  motionConstraint_.computeConstantPart();

  computeConstantPart();
}

template <typename Scalar>
void TrajectoryWalkgen<Scalar>::setSamplingPeriod(Scalar samplingPeriod)
{
  assert(samplingPeriod>0.0);

  noDynModel_.setSamplingPeriod(samplingPeriod);

  jerkMinObj_.computeConstantPart();
  velTrackingObj_.computeConstantPart();
  posTrackingObj_.computeConstantPart();
  motionConstraint_.computeConstantPart();

  computeConstantPart();
}

template <typename Scalar>
void TrajectoryWalkgen<Scalar>::setVelRefInWorldFrame(const VectorX& velRef)
{
  assert(velRef==velRef);
  assert(velRef.size()==noDynModel_.getNbSamples());
  velTrackingObj_.setVelRefInWorldFrame(velRef);
}

template <typename Scalar>
void TrajectoryWalkgen<Scalar>::setPosRefInWorldFrame(const VectorX& posRef)
{
  assert(posRef==posRef);
  assert(posRef.size()==noDynModel_.getNbSamples());
  posTrackingObj_.setPosRefInWorldFrame(posRef);
}

template <typename Scalar>
void TrajectoryWalkgen<Scalar>::setVelLimit(Scalar limit)
{
  assert(limit>=0);

  noDynModel_.setVelocityLimit(limit);
}

template <typename Scalar>
void TrajectoryWalkgen<Scalar>::setAccLimit(Scalar limit)
{
  assert(limit>=0);

  noDynModel_.setAccelerationLimit(limit);
}

template <typename Scalar>
void TrajectoryWalkgen<Scalar>::setJerkLimit(Scalar limit)
{
  assert(limit>=0);

  noDynModel_.setJerkLimit(limit);
}

template <typename Scalar>
void TrajectoryWalkgen<Scalar>::setState(const VectorX& state)
{
  assert(state==state);
  assert(state.size()==3);

  noDynModel_.setState(state);
}

template <typename Scalar>
void TrajectoryWalkgen<Scalar>::setWeightings(const  TrajectoryWalkgenWeighting<Scalar>& weighting)
{
  assert(weighting.velocityTracking>=0);
  assert(weighting.velocityTracking>=0);
  assert(weighting.jerkMinimization>=0);

  weighting_ = weighting;

  computeConstantPart();
}

template <typename Scalar>
void TrajectoryWalkgen<Scalar>::setConfig(const TrajectoryWalkgenConfig<Scalar>& config)
{
  assert(config.withMotionConstraints == config.withMotionConstraints);

  config_ = config;

  computeConstantPart();
}

template <typename Scalar>
bool TrajectoryWalkgen<Scalar>::solve(Scalar feedBackPeriod)
{
  int N = noDynModel_.getNbSamples();
  int M = config_.withMotionConstraints? motionConstraint_.getNbConstraints() : 0;

  assert(velTrackingObj_.getGradient(X_).size() == N);
  assert(posTrackingObj_.getGradient(X_).size() == N);
  assert(jerkMinObj_.getGradient(X_).size() == N);

  if (config_.withMotionConstraints)
  {
    assert(motionConstraint_.getFunctionInf(X_).size() == M);
    assert(motionConstraint_.getFunctionSup(X_).size() == M);
  }

  assert(feedBackPeriod>0);

  qpMatrix_.p.fill(Scalar(0.0));
  qpMatrix_.bu.fill(Scalar(10e10));
  qpMatrix_.bl.fill(Scalar(-10e10));
  qpMatrix_.xu.fill(Scalar(10e10));
  qpMatrix_.xl.fill(Scalar(-10e10));

  if (weighting_.velocityTracking>0.0)
  {
    qpMatrix_.p +=
       weighting_.velocityTracking*velTrackingObj_.getGradient(X_);
  }
  if (weighting_.positionTracking>0.0)
  {
    qpMatrix_.p +=
        weighting_.positionTracking*posTrackingObj_.getGradient(X_);
  }
  if (weighting_.jerkMinimization>0.0)
  {
    qpMatrix_.p += weighting_.jerkMinimization*jerkMinObj_.getGradient(X_);
  }

  if (config_.withMotionConstraints)
  {
    qpMatrix_.bl.segment(0, M) = motionConstraint_.getFunctionInf(X_);
    qpMatrix_.bu.segment(0, M) = motionConstraint_.getFunctionSup(X_);

    qpMatrix_.xu.fill(noDynModel_.getJerkLimit());
    qpMatrix_.xu -= X_;

    qpMatrix_.xl.fill(-noDynModel_.getJerkLimit());
    qpMatrix_.xl-= X_;

  }

  bool solutionFound = qpoasesSolver_->solve(qpMatrix_, dX_, true);

  if (!solutionFound)
  {
    std::cerr << "Q : " << std::endl << qpMatrix_.Q << std::endl;
    std::cerr << "p : " << qpMatrix_.p.transpose() << std::endl;
    std::cerr << "A : " << std::endl << qpMatrix_.A << std::endl;
    std::cerr << "bl: " << qpMatrix_.bl.transpose() << std::endl;
    std::cerr << "bu: " << qpMatrix_.bu.transpose() << std::endl;
    std::cerr << "X : " << X_.transpose() << std::endl;
    std::cerr << "dX: " << dX_.transpose() << std::endl;
    std::cerr << "c : " << noDynModel_.getState() << std::endl;
  }

  X_ += dX_;


  noDynModel_.updateState(X_(0), feedBackPeriod);

  return solutionFound;
}


template <typename Scalar>
const typename Type<Scalar>::VectorX& TrajectoryWalkgen<Scalar>::getState() const
{
  return noDynModel_.getState();
}

template <typename Scalar>
const Scalar TrajectoryWalkgen<Scalar>::getJerk() const
{
  return X_(0);
}

template <typename Scalar>
void TrajectoryWalkgen<Scalar>::computeConstantPart()
{
  int N = noDynModel_.getNbSamples();
  int M1 = config_.withMotionConstraints? motionConstraint_.getNbConstraints() : 0;
  int M = M1;

  assert(velTrackingObj_.getHessian().rows() == N);
  assert(velTrackingObj_.getHessian().cols() == N);

  assert(posTrackingObj_.getHessian().rows() == N);
  assert(posTrackingObj_.getHessian().cols() == N);

  assert(jerkMinObj_.getHessian().rows() == N);
  assert(jerkMinObj_.getHessian().cols() == N);

  if (config_.withMotionConstraints)
  {
    assert(motionConstraint_.getGradient().cols() == N);
    assert(motionConstraint_.getGradient().rows() == M1);
  }

  qpoasesSolver_.reset(makeQPSolver<Scalar>(N, M));

  qpMatrix_.Q.setZero(N, N);
  qpMatrix_.p.setZero(N, 1);
  qpMatrix_.A.setZero(M, N);
  qpMatrix_.bl.setZero(M, 1);
  qpMatrix_.bu.setZero(M, 1);
  qpMatrix_.xl.setZero(N, 1);
  qpMatrix_.xu.setZero(N, 1);

  if (weighting_.velocityTracking>0.0)
  {
    qpMatrix_.Q += weighting_.velocityTracking*velTrackingObj_.getHessian();
  }
  if (weighting_.positionTracking>0.0)
  {
    qpMatrix_.Q += weighting_.positionTracking*posTrackingObj_.getHessian();
  }
  if (weighting_.jerkMinimization>0.0)
  {
    qpMatrix_.Q += weighting_.jerkMinimization*jerkMinObj_.getHessian();
  }

  if (config_.withMotionConstraints)
  {
    qpMatrix_.A.block(0, 0, M1, N) = motionConstraint_.getGradient();
  }
}


  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(TrajectoryWalkgen);

}
