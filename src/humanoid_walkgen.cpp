////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_walkgen.cpp
///\brief Main program for Humanoid
///\author de Gourcuff Martin
///\date 11/07/13
///
////////////////////////////////////////////////////////////////////////////////

#include "humanoid_walkgen.h"

namespace MPCWalkgen
{

  //TODO: MAke leftFootModel and RightFootModel disappear
  HumanoidWalkgen::HumanoidWalkgen()
    :feetSupervisor_(leftFootModel_, rightFootModel_)
    ,velTrackingObj_(lipModel_, feetSupervisor_)
    ,jerkMinObj_(lipModel_, feetSupervisor_)
    ,copCenteringObj_(lipModel_, feetSupervisor_)
    ,copConstraint_(lipModel_, feetSupervisor_)
    ,footConstraint_(lipModel_, feetSupervisor_)
    ,weighting_()
    ,config_()
    ,maximumNbOfConstraints_(0)
    ,maximumNbOfSteps_(0)
  {
    dX_.setZero(2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());
    X_.setZero(2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    computeConstantPart();
  }

  HumanoidWalkgen::~HumanoidWalkgen(){}

  void HumanoidWalkgen::setMaximumNbOfSteps(int maximumNbOfSteps)
  {
    assert(maximumNbOfSteps>0);

    maximumNbOfSteps_ = maximumNbOfSteps;

    computeConstantPart();
  }


  void HumanoidWalkgen::setNbSamples(int nbSamples)
  {
    assert(nbSamples>0);
    assert(static_cast<Scalar>(nbSamples)
           *feetSupervisor_.getSamplingPeriod()/feetSupervisor_.getStepPeriod()
           < maximumNbOfSteps_);

    lipModel_.setNbSamples(nbSamples);
    feetSupervisor_.setNbSamples(nbSamples);

    copConstraint_.computeConstantPart();

    computeConstantPart();
  }

  void HumanoidWalkgen::setSamplingPeriod(Scalar samplingPeriod)
  {
    assert(samplingPeriod>=0);
    assert(static_cast<Scalar>(feetSupervisor_.getNbSamples())
           *samplingPeriod/feetSupervisor_.getStepPeriod()
           < maximumNbOfSteps_);

    lipModel_.setSamplingPeriod(samplingPeriod);
    feetSupervisor_.setSamplingPeriod(samplingPeriod);

    copConstraint_.computeConstantPart();

    computeConstantPart();
  }

  void HumanoidWalkgen::setStepPeriod(Scalar stepPeriod)
  {
    assert(stepPeriod>0.0);
    assert(static_cast<Scalar>(feetSupervisor_.getNbSamples())
           *feetSupervisor_.getSamplingPeriod()/stepPeriod
           < maximumNbOfSteps_);

    feetSupervisor_.setStepPeriod(stepPeriod);
  }

  void HumanoidWalkgen::setLeftFootKinematicConvexPolygon(const ConvexPolygon& convexPolygon)
  {
    feetSupervisor_.setLeftFootKinematicConvexPolygon(convexPolygon);
  }

  void HumanoidWalkgen::setRightFootKinematicConvexPolygon(const ConvexPolygon &convexPolygon)
  {
    feetSupervisor_.setRightFootKinematicConvexPolygon(convexPolygon);
  }

  void HumanoidWalkgen::setLeftFootCopConvexPolygon(const ConvexPolygon& convexPolygon)
  {
    feetSupervisor_.setLeftFootCopConvexPolygon(convexPolygon);
  }

  void HumanoidWalkgen::setRightFootCopConvexPolygon(const ConvexPolygon& convexPolygon)
  {
    feetSupervisor_.setRightFootCopConvexPolygon(convexPolygon);
  }

  void HumanoidWalkgen::setVelRefInWorldFrame(const VectorX& velRef)
  {
    assert(velRef==velRef);
    assert(velRef.size()==2*lipModel_.getNbSamples());
    velTrackingObj_.setVelRefInWorldFrame(velRef);
  }


  //TODO: for all state setters, add the 2nd or 4th element (=1)
  //Makes the public API more intuitive
  void HumanoidWalkgen::setLeftFootStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    feetSupervisor_.setLeftFootStateX(state);
  }

  void HumanoidWalkgen::setLeftFootStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    feetSupervisor_.setLeftFootStateY(state);
  }

  void HumanoidWalkgen::setLeftFootStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    feetSupervisor_.setLeftFootStateZ(state);
  }

  void HumanoidWalkgen::setRightFootStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    feetSupervisor_.setRightFootStateX(state);
  }

  void HumanoidWalkgen::setRightFootStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    feetSupervisor_.setRightFootStateY(state);
  }

  void HumanoidWalkgen::setRightFootStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    feetSupervisor_.setRightFootStateZ(state);
  }

  void HumanoidWalkgen::setComStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==4);
    assert(state(3)==1.0);
    lipModel_.setStateX(state);
  }

  void HumanoidWalkgen::setComStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==4);
    assert(state(3)==1.0);
    lipModel_.setStateY(state);
  }

  void HumanoidWalkgen::setComStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==4);
    assert(state(1)==0.0);
    assert(state(2)==0.0);
    assert(state(3)==1.0);
    lipModel_.setComHeight(state(0));
  }

  const VectorX& HumanoidWalkgen::getLeftFootStateX() const
  {
    return leftFootModel_.getStateX();
  }

  const VectorX& HumanoidWalkgen::getLeftFootStateY() const
  {
    return leftFootModel_.getStateY();
  }

  const VectorX& HumanoidWalkgen::getLeftFootStateZ() const
  {
    return leftFootModel_.getStateZ();
  }

  const VectorX& HumanoidWalkgen::getRightFootStateX() const
  {
    return rightFootModel_.getStateX();
  }

  const VectorX& HumanoidWalkgen::getRightFootStateY() const
  {
    return rightFootModel_.getStateY();
  }
  const VectorX& HumanoidWalkgen::getRightFootStateZ() const
  {
    return rightFootModel_.getStateZ();
  }

  const VectorX& HumanoidWalkgen::getComStateX() const
  {
    return lipModel_.getStateX();
  }

  const VectorX& HumanoidWalkgen::getComStateY() const
  {
    return lipModel_.getStateY();
  }

  const VectorX& HumanoidWalkgen::getComStateZ() const
  {
    return lipModel_.getStateZ();
  }

  void HumanoidWalkgen::setLeftFootMaxHeight(
      Scalar leftFootMaxHeight)
  {
    feetSupervisor_.setLeftFootMaxHeight(
          leftFootMaxHeight);
  }

  void HumanoidWalkgen::setRightFootMaxHeight(
      Scalar rightFootMaxHeight)
  {
    feetSupervisor_.setRightFootMaxHeight(
          rightFootMaxHeight);
  }

  void HumanoidWalkgen::setLeftFootYawUpperBound(Scalar leftFootYawUpperBound)
  {
    feetSupervisor_.setLeftFootYawUpperBound(
          leftFootYawUpperBound);
  }
  void HumanoidWalkgen::setLeftFootYawLowerBound(Scalar leftFootYawLowerBound)
  {
    feetSupervisor_.setLeftFootYawLowerBound(
          leftFootYawLowerBound);
  }
  void HumanoidWalkgen::setRightFootYawUpperBound(Scalar rightFootYawUpperBound)
  {
    feetSupervisor_.setRightFootYawUpperBound(
          rightFootYawUpperBound);
  }
  void HumanoidWalkgen::setRightFootYawLowerBound(Scalar rightFootYawLowerBound)
  {
    feetSupervisor_.setRightFootYawLowerBound(
          rightFootYawLowerBound);
  }

  void HumanoidWalkgen::setLeftFootYawSpeedUpperBound(
      Scalar leftFootYawSpeedUpperBound)
  {
    feetSupervisor_.setLeftFootYawSpeedUpperBound(
          leftFootYawSpeedUpperBound);
  }
  void HumanoidWalkgen::setRightFootYawSpeedUpperBound(
      Scalar rightFootYawSpeedUpperBound)
  {
    feetSupervisor_.setRightFootYawSpeedUpperBound(
          rightFootYawSpeedUpperBound);
  }

  void HumanoidWalkgen::setLeftFootYawAccelerationUpperBound(
      Scalar leftFootYawAccelerationUpperBound)
  {
    feetSupervisor_.setLeftFootYawAccelerationUpperBound(
          leftFootYawAccelerationUpperBound);
  }
  void HumanoidWalkgen::setRightFootYawAccelerationUpperBound(
      Scalar rightFootYawAccelerationUpperBound)
  {
    feetSupervisor_.setRightFootYawAccelerationUpperBound(
          rightFootYawAccelerationUpperBound);
  }

  void HumanoidWalkgen::setWeightings(const HumanoidWalkgenImpl::Weighting& weighting)
  {
    assert(weighting_.velocityTracking >=0);
    assert(weighting_.copCentering >=0);
    assert(weighting_.jerkMinimization >=0);
    weighting_ = weighting;

    computeConstantPart();
  }

  void HumanoidWalkgen::setConfig(const HumanoidWalkgenImpl::Config& config)
  {
    config_ = config;

    computeConstantPart();
  }

  //TODO: Compute normalization factors
  bool HumanoidWalkgen::solve(Scalar feedBackPeriod)
  {
    assert(feedBackPeriod>0);

    int N = lipModel_.getNbSamples();
    int M = feetSupervisor_.getNbPreviewedSteps();
    int sizeVec = 2*N + 2*M;
    int nbCtrCop = config_.withCopConstraints? copConstraint_.getNbConstraints() : 0;
    int nbCtrFoot = config_.withFeetConstraints? footConstraint_.getNbConstraints() : 0;
    int nbCtr = nbCtrCop + nbCtrFoot;
    int index = M*maximumNbOfConstraints_ + nbCtr;

    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());

    //Setting matrix Q and vector p
    if (weighting_.velocityTracking>0.0)
    {
      qpMatrixVec_[index].Q +=
          weighting_.velocityTracking*velTrackingObj_.getHessian();

      qpMatrixVec_[index].p +=
          weighting_.velocityTracking*velTrackingObj_.getGradient(X_);
    }

    if (weighting_.jerkMinimization>0.0)
    {
      qpMatrixVec_[index].Q +=
          weighting_.jerkMinimization*jerkMinObj_.getHessian();

      qpMatrixVec_[index].p +=
          weighting_.jerkMinimization*jerkMinObj_.getGradient(X_);
    }

    if (weighting_.copCentering>0.0)
    {
      qpMatrixVec_[index].Q +=
          weighting_.copCentering*copCenteringObj_.getHessian();

      qpMatrixVec_[index].p +=
          weighting_.copCentering*copCenteringObj_.getGradient(X_);
    }

    //Setting matrix A, vector b and vectors xl and xu
    if (config_.withCopConstraints)
    {
      qpMatrixVec_[index].A.block(0, 0, nbCtrCop, sizeVec)
          += copConstraint_.getGradient(X_.rows());

      qpMatrixVec_[index].bu.segment(0 ,nbCtrCop).
          cwiseMin(-copConstraint_.getFunction(X_));
      qpMatrixVec_[index].xl.segment(0, 2*N).
          cwiseMax(copConstraint_.getInfBounds(X_));
      qpMatrixVec_[index].xu.segment(0, 2*N).
          cwiseMin(copConstraint_.getSupBounds(X_));
    }

    if (config_.withFeetConstraints)
    {
      qpMatrixVec_[index].A.block(nbCtrCop, sizeVec, nbCtrFoot, sizeVec)
          += footConstraint_.getGradient(X_.rows());

      qpMatrixVec_[index].bu.segment(nbCtrCop, nbCtrFoot).
          cwiseMin(-footConstraint_.getFunction(X_));
      qpMatrixVec_[index].xl.segment(2*N, 2*M).
          cwiseMax(footConstraint_.getInfBounds(X_));
      qpMatrixVec_[index].xu.segment(2*N, 2*M).
          cwiseMin(footConstraint_.getSupBounds(X_));
    }

    //Setting matrix At
    qpMatrixVec_[index].At = qpMatrixVec_[index].A.transpose();

    //Normalization

    bool solutionFound = qpoasesSolverVec_[index].solve(qpMatrixVec_[index], dX_, true);
    X_ += dX_;

    //Transforming solution
    convertCopInLFtoJerkInWF();

    //Updating states
    lipModel_.updateStateX(transformedX_(0), feedBackPeriod);
    lipModel_.updateStateY(transformedX_(N), feedBackPeriod);

    //TODO: Updating feet supervisor

    return solutionFound;
  }


  void HumanoidWalkgen::computeConstantPart()
  {
    maximumNbOfConstraints_ = feetSupervisor_.getMaximumNbOfCopConstraints()
        + feetSupervisor_.getMaximumNbOfKinematicConstraints();

    qpoasesSolverVec_.resize(maximumNbOfSteps_*maximumNbOfConstraints_);
    qpMatrixVec_.resize(maximumNbOfSteps_*maximumNbOfConstraints_);

    int nbVariables = 2*lipModel_.getNbSamples();

    for(int i=0; i<maximumNbOfSteps_; ++i)
    {
      for(int j=0; j<maximumNbOfConstraints_; ++j)
      {
        nbVariables = 2*lipModel_.getNbSamples() + 2*i;

        qpoasesSolverVec_[i*maximumNbOfConstraints_ + j] = QPOasesSolver(nbVariables, j);
        qpMatrixVec_[i*maximumNbOfConstraints_ + j].Q.setZero(nbVariables, nbVariables);
        qpMatrixVec_[i*maximumNbOfConstraints_ + j].p.setZero(nbVariables);

        qpMatrixVec_[i*maximumNbOfConstraints_ + j].A.setZero(j, nbVariables);
        qpMatrixVec_[i*maximumNbOfConstraints_ + j].bl.setConstant(j,
                                                                   -MAXIMUM_BOUND_VALUE);
        qpMatrixVec_[i*maximumNbOfConstraints_ + j].bu.setConstant(j,
                                                                   MAXIMUM_BOUND_VALUE);

        qpMatrixVec_[i*maximumNbOfConstraints_ + j].xl.setConstant(nbVariables,
                                                                   -MAXIMUM_BOUND_VALUE);
        qpMatrixVec_[i*maximumNbOfConstraints_ + j].xu.setConstant(nbVariables,
                                                                   MAXIMUM_BOUND_VALUE);
      }
    }
  }


  void HumanoidWalkgen::convertCopInLFtoJerkInWF()
  {
    int N = lipModel_.getNbSamples();
    int M = feetSupervisor_.getNbPreviewedSteps();

    const MatrixX& footPosU = feetSupervisor_.getFeetPosLinearDynamic().U;
    const MatrixX& rot = feetSupervisor_.getRotationMatrixT();

    transformedX_.segment(0, N) = rot.block(0, 0, N, N)*X_.segment(0, N)
        + rot.block(0, N, N, N)*X_.segment(N, N);
    transformedX_.segment(N, N) = rot.block(0, N, N, N)*X_.segment(0, N)
        + rot.block(N, N, N, N)*X_.segment(N, N);

    transformedX_.segment(0, N) += footPosU*X_.segment(2*N, M);
    transformedX_.segment(N, N) += footPosU*X_.segment(2*N + M, M);

    transformedX_.segment(0, N) -= lipModel_.getCopXLinearDynamic().S*lipModel_.getStateX();
    transformedX_.segment(N, N) -= lipModel_.getCopYLinearDynamic().S*lipModel_.getStateY();

    transformedX_.segment(0, N) *= lipModel_.getCopXLinearDynamic().Uinv;
    transformedX_.segment(N, N) *= lipModel_.getCopYLinearDynamic().Uinv;
  }

}
