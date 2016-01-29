////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_walkgen.cpp
///\brief Main program for Humanoid
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/humanoid_walkgen.h>
#include "macro.h"
#include <mpc-walkgen/constant.h>

namespace MPCWalkgen
{

  //TODO: Make leftFootModel and RightFootModel disappear
  template <typename Scalar>
  HumanoidWalkgen<Scalar>::HumanoidWalkgen()
    :feetSupervisor_()
    ,velTrackingObj_(lipModel_, feetSupervisor_)
    ,jerkMinObj_(lipModel_, feetSupervisor_)
    ,copCenteringObj_(lipModel_, feetSupervisor_)
    ,copConstraint_(lipModel_, feetSupervisor_)
    ,footConstraint_(lipModel_, feetSupervisor_)
    ,weighting_()
    ,config_()
    ,maximumNbOfConstraints_(0)
    ,maximumNbOfSteps_(0)
    ,move_(false)
    ,firstCallSinceLastDS_(true)
  {
    const int sizeVec = 2*lipModel_.getNbSamples() +
                        2*feetSupervisor_.getNbPreviewedSteps();
    dX_.setZero(sizeVec);
    X_.setZero(sizeVec);

    computeConstantPart();
  }

  template <typename Scalar>
  HumanoidWalkgen<Scalar>::~HumanoidWalkgen(){}

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setNbSamples(int nbSamples)
  {
    assert(nbSamples>0);

    lipModel_.setNbSamples(nbSamples);
    feetSupervisor_.setNbSamples(nbSamples);

    copConstraint_.computeConstantPart();

    //TODO: change
    // Updating QP variable sizes
    dX_.setZero(2*lipModel_.getNbSamples() +
                2*feetSupervisor_.getNbPreviewedSteps());
    X_.setZero(2*feetSupervisor_.getNbSamples() +
               2*feetSupervisor_.getNbPreviewedSteps());

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setSamplingPeriod(Scalar samplingPeriod)
  {
    assert(samplingPeriod>=0);

    lipModel_.setSamplingPeriod(samplingPeriod);
    feetSupervisor_.setSamplingPeriod(samplingPeriod);

    copConstraint_.computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setStepPeriod(Scalar stepPeriod)
  {
    assert(stepPeriod>0.0);
    assert(stepPeriod - feetSupervisor_.getSamplingPeriod() + Constant<Scalar>::EPSILON > 0);

    feetSupervisor_.setStepPeriod(stepPeriod);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setInitialDoubleSupportLength(
      Scalar initialDoubleSupportLength)
  {
    feetSupervisor_.setInitialDoubleSupportLength(
          initialDoubleSupportLength);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setLeftFootKinematicConvexPolygon(
      const ConvexPolygon<Scalar>& convexPolygon)
  {
    feetSupervisor_.setLeftFootKinematicConvexPolygon(convexPolygon);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setRightFootKinematicConvexPolygon(
      const ConvexPolygon<Scalar> &convexPolygon)
  {
    feetSupervisor_.setRightFootKinematicConvexPolygon(convexPolygon);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setLeftFootCopConvexPolygon(
      const ConvexPolygon<Scalar>& convexPolygon)
  {
    feetSupervisor_.setLeftFootCopConvexPolygon(convexPolygon);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setRightFootCopConvexPolygon(
      const ConvexPolygon<Scalar>& convexPolygon)
  {
    feetSupervisor_.setRightFootCopConvexPolygon(convexPolygon);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setVelRefInWorldFrame(const VectorX& velRef)
  {
    assert(velRef==velRef);
    assert(velRef.size()==2*lipModel_.getNbSamples());
    velTrackingObj_.setVelRefInWorldFrame(velRef);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setAngularVelRefInWorldFrame(const VectorX& angularVelRef)
  {
    assert(angularVelRef==angularVelRef);
    assert(angularVelRef.size()==lipModel_.getNbSamples());
    //TODO: complete
    //velTrackingObj_.setAngularVelRefInWorldFrame(angularVelRef);
  }


  //TODO: for all state setters, add the 2nd or 4th element (=1)
  //Makes the public API more intuitive
  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setLeftFootStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    feetSupervisor_.setLeftFootStateX(state);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setLeftFootStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    feetSupervisor_.setLeftFootStateY(state);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setLeftFootStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    feetSupervisor_.setLeftFootStateZ(state);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setRightFootStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    feetSupervisor_.setRightFootStateX(state);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setRightFootStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    feetSupervisor_.setRightFootStateY(state);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setRightFootStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    feetSupervisor_.setRightFootStateZ(state);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setComStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    lipModel_.setStateX(state);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setComStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    lipModel_.setStateY(state);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setComStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    assert(state(1)==0.0);
    assert(state(2)==0.0);
    lipModel_.setComHeight(state(0));
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setLeftFootMaxHeight(Scalar leftFootMaxHeight)
  {
    feetSupervisor_.setLeftFootMaxHeight(leftFootMaxHeight);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setRightFootMaxHeight(Scalar rightFootMaxHeight)
  {
    feetSupervisor_.setRightFootMaxHeight(rightFootMaxHeight);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setLeftFootYawUpperBound(Scalar leftFootYawUpperBound)
  {
    feetSupervisor_.setLeftFootYawUpperBound(
          leftFootYawUpperBound);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setLeftFootYawLowerBound(Scalar leftFootYawLowerBound)
  {
    feetSupervisor_.setLeftFootYawLowerBound(
          leftFootYawLowerBound);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setRightFootYawUpperBound(Scalar rightFootYawUpperBound)
  {
    feetSupervisor_.setRightFootYawUpperBound(
          rightFootYawUpperBound);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setRightFootYawLowerBound(Scalar rightFootYawLowerBound)
  {
    feetSupervisor_.setRightFootYawLowerBound(
          rightFootYawLowerBound);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setLeftFootYawSpeedUpperBound(
      Scalar leftFootYawSpeedUpperBound)
  {
    feetSupervisor_.setLeftFootYawSpeedUpperBound(
          leftFootYawSpeedUpperBound);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setRightFootYawSpeedUpperBound(
      Scalar rightFootYawSpeedUpperBound)
  {
    feetSupervisor_.setRightFootYawSpeedUpperBound(
          rightFootYawSpeedUpperBound);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setLeftFootYawAccelerationUpperBound(
      Scalar leftFootYawAccelerationUpperBound)
  {
    feetSupervisor_.setLeftFootYawAccelerationUpperBound(
          leftFootYawAccelerationUpperBound);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setRightFootYawAccelerationUpperBound(
      Scalar rightFootYawAccelerationUpperBound)
  {
    feetSupervisor_.setRightFootYawAccelerationUpperBound(
          rightFootYawAccelerationUpperBound);
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setWeightings(const HumanoidWalkgenWeighting<Scalar>& weighting)
  {
    assert(weighting_.velocityTracking >=0);
    assert(weighting_.copCentering >=0);
    assert(weighting_.jerkMinimization >=0);

    weighting_ = weighting;
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setConfig(const HumanoidWalkgenConfig<Scalar>& config)
  {
    config_ = config;

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::setMove(bool move)
  {
    move_ = move;
    feetSupervisor_.setMove(move);
    if(feetSupervisor_.isInDS())
    {
      firstCallSinceLastDS_ = true;
    }
  }

  template <typename Scalar>
  bool HumanoidWalkgen<Scalar>::solve(Scalar feedBackPeriod)
  {
    //Updating the supervisor timeline
    feetSupervisor_.updateTimeline(X_, feedBackPeriod);

    if(std::abs(feedBackPeriod - lipModel_.getFeedbackPeriod()) > Constant<Scalar>::EPSILON)
    {
      lipModel_.setFeedbackPeriod(feedBackPeriod);
    }

    assert(feedBackPeriod>0);
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());

    int N = lipModel_.getNbSamples();
    int M = feetSupervisor_.getNbPreviewedSteps();
    int sizeVec = 2*N + 2*M;
    int nbCtrCop = config_.withCopConstraints? copConstraint_.getNbConstraints() : 0;
    int nbCtrFoot = config_.withFeetConstraints? footConstraint_.getNbConstraints() : 0;
    int nbCtr = nbCtrCop + nbCtrFoot;
    int index = M*(maximumNbOfConstraints_ + 1) + nbCtr;

    // Filling the QP variable for the very beginning of the algorithm
    if(firstCallSinceLastDS_)
    {
      Scalar copInitialPosXinWF =
          getComStateX()(0) - getComStateZ()(0)*getComStateX()(2)/Constant<Scalar>::GRAVITY_NORM;
      Scalar copInitialPosYinWF =
          getComStateY()(0) - getComStateZ()(0)*getComStateY()(2)/Constant<Scalar>::GRAVITY_NORM;

      const MatrixX& rot(feetSupervisor_.getRotationMatrix());

      X_.segment(0, N) =
          rot.block(0,0,N,N)*VectorX::Constant(N, copInitialPosXinWF - getLeftFootStateX()(0)) +
          rot.block(0,N,N,N)*VectorX::Constant(N, copInitialPosYinWF - getLeftFootStateY()(0));

      X_.segment(N, N) =
          rot.block(N,0,N,N)*VectorX::Constant(N, copInitialPosXinWF - getLeftFootStateX()(0)) +
          rot.block(N,N,N,N)*VectorX::Constant(N, copInitialPosYinWF - getLeftFootStateY()(0));

      firstCallSinceLastDS_ = false;
    }


    // Choosing the QP Matrices with proper size
    QPMatrices<Scalar>& qpMatrices = qpMatrixVec_[index];

    qpMatrices.Q.fill(0.0);
    qpMatrices.p.fill(0.0);
    qpMatrices.A.fill(0.0);
    qpMatrices.bl.fill(-Constant<Scalar>::MAXIMUM_BOUND_VALUE);
    qpMatrices.bu.fill(Constant<Scalar>::MAXIMUM_BOUND_VALUE);
    qpMatrices.xl.fill(-Constant<Scalar>::MAXIMUM_BOUND_VALUE);
    qpMatrices.xu.fill(Constant<Scalar>::MAXIMUM_BOUND_VALUE);

    // Setting matrix Q and vector p
    if (weighting_.velocityTracking>0.0)
    {
      qpMatrices.Q +=
          weighting_.velocityTracking*velTrackingObj_.getHessian();

      qpMatrices.p +=
          weighting_.velocityTracking*velTrackingObj_.getGradient(X_);
    }

    if (weighting_.jerkMinimization>0.0)
    {
      qpMatrices.Q +=
          weighting_.jerkMinimization*jerkMinObj_.getHessian();

      qpMatrices.p +=
          weighting_.jerkMinimization*jerkMinObj_.getGradient(X_);
    }

    if (weighting_.copCentering>0.0)
    {
      qpMatrices.Q +=
          weighting_.copCentering*copCenteringObj_.getHessian();

      qpMatrices.p +=
          weighting_.copCentering*copCenteringObj_.getGradient(X_);
    }

    //Setting matrix A, vector b, vector xl and vector xu
    if (config_.withCopConstraints)
    {
      qpMatrices.A.block(0, 0, nbCtrCop, sizeVec)
          += copConstraint_.getGradient(X_.rows());

      qpMatrices.bu.segment(0 ,nbCtrCop) =
          qpMatrices.bu.segment(0 ,nbCtrCop).cwiseMin(-copConstraint_.getFunction(X_));
      qpMatrices.xl.segment(0, 2*N) =
          qpMatrices.xl.segment(0, 2*N).cwiseMax(copConstraint_.getInfBounds(X_));
      qpMatrices.xu.segment(0, 2*N) =
          qpMatrices.xu.segment(0, 2*N).cwiseMin(copConstraint_.getSupBounds(X_));
    }

    if (config_.withFeetConstraints)
    {
      qpMatrices.A.block(nbCtrCop, 0, nbCtrFoot, sizeVec)
          += footConstraint_.getGradient(X_.rows());

      qpMatrices.bu.segment(nbCtrCop, nbCtrFoot) =
          qpMatrices.bu.segment(nbCtrCop, nbCtrFoot).cwiseMin(-footConstraint_.getFunction(X_));
      qpMatrices.xl.segment(2*N, 2*M) =
          qpMatrices.xl.segment(2*N, 2*M).cwiseMax(footConstraint_.getInfBounds(X_));
      qpMatrices.xu.segment(2*N, 2*M) =
          qpMatrices.xu.segment(2*N, 2*M).cwiseMin(footConstraint_.getSupBounds(X_));
    }

    //Normalization of the matrices. The smallest element value of the QP matrices is at least one.
    qpMatrices.normalizeMatrices(Constant<Scalar>::EPSILON);

    //Setting matrix At
    qpMatrices.At = qpMatrices.A.transpose();

    dX_.resize(sizeVec);

    bool solutionFound = qpoasesSolverVec_[index].solve(qpMatrices, dX_, false);

    X_ += dX_;

    //Transforming solution
    convertCopInLFtoComJerk();

    //Updating states
    lipModel_.updateStateX(transformedX_(0), feedBackPeriod);
    lipModel_.updateStateY(transformedX_(N), feedBackPeriod);

    //Updating feet supervisor
    feetSupervisor_.updateFeetStates(
          transformedX_.segment(2*feetSupervisor_.getNbSamples(),
                                2*feetSupervisor_.getNbPreviewedSteps()),
          feedBackPeriod);


    //display("/home/mdegourcuff/Bureau/Test_new_MPCWalkgen/QPSol.txt");

    return solutionFound;
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::computeConstantPart()
  {
    // Updating qpoasesSolverVec_ and qpMatrixVec_
    maximumNbOfSteps_ = feetSupervisor_.getNbSamples();
    if(config_.withCopConstraints)
    {
      maximumNbOfConstraints_ = feetSupervisor_.getMaximumNbOfCopConstraints()
          *feetSupervisor_.getNbSamples();
    }
    if(config_.withFeetConstraints)
    {
      maximumNbOfConstraints_ += feetSupervisor_.getMaximumNbOfKinematicConstraints()
          *maximumNbOfSteps_;
    }

    qpoasesSolverVec_.reserve((maximumNbOfSteps_ + 1)*(maximumNbOfConstraints_ + 1));
    qpMatrixVec_.resize((maximumNbOfSteps_ + 1)*(maximumNbOfConstraints_ + 1));

    int nbVariables = 2*lipModel_.getNbSamples();

    for(int i=0; i<maximumNbOfSteps_ + 1; ++i)
    {
      for(int j=0; j<maximumNbOfConstraints_ + 1; ++j)
      {
        nbVariables = 2*lipModel_.getNbSamples() + 2*i;

        qpoasesSolverVec_.push_back(makeQPSolver<Scalar>(nbVariables, j));
        qpMatrixVec_[i*(maximumNbOfConstraints_ + 1) + j].Q.setZero(nbVariables, nbVariables);
        qpMatrixVec_[i*(maximumNbOfConstraints_ + 1) + j].p.setZero(nbVariables);

        qpMatrixVec_[i*(maximumNbOfConstraints_ + 1) + j].A.setZero(j, nbVariables);
        qpMatrixVec_[i*(maximumNbOfConstraints_ + 1) + j].bl.setConstant(j,
                                                            -Constant<Scalar>::MAXIMUM_BOUND_VALUE);
        qpMatrixVec_[i*(maximumNbOfConstraints_ + 1) + j].bu.setConstant(j,
                                                            Constant<Scalar>::MAXIMUM_BOUND_VALUE);

        qpMatrixVec_[i*(maximumNbOfConstraints_ + 1) + j].xl.setConstant(nbVariables,
                                                            -Constant<Scalar>::MAXIMUM_BOUND_VALUE);
        qpMatrixVec_[i*(maximumNbOfConstraints_ + 1) + j].xu.setConstant(nbVariables,
                                                            Constant<Scalar>::MAXIMUM_BOUND_VALUE);
      }
    }
  }

  template <typename Scalar>
  void HumanoidWalkgen<Scalar>::convertCopInLFtoComJerk()
  {
    int N = lipModel_.getNbSamples();
    int M = feetSupervisor_.getNbPreviewedSteps();

    transformedX_ = X_;

    int nb = feetSupervisor_.getNbOfCallsBeforeNextSample() - 1;

    const MatrixX& footPosU = feetSupervisor_.getFeetPosLinearDynamic().U;
    const MatrixX& footPosS = feetSupervisor_.getFeetPosLinearDynamic().S;
    const LinearDynamic<Scalar>& dynCopX = lipModel_.getCopXLinearDynamic(nb);
    const LinearDynamic<Scalar>& dynCopY = lipModel_.getCopYLinearDynamic(nb);
    const MatrixX& rotT = feetSupervisor_.getRotationMatrixT();

    transformedX_.segment(0, N) = rotT.block(0, 0, N, N)*X_.segment(0, N)
        + rotT.block(0, N, N, N)*X_.segment(N, N);
    transformedX_.segment(N, N) = rotT.block(0, N, N, N)*X_.segment(0, N)
        + rotT.block(N, N, N, N)*X_.segment(N, N);

    transformedX_.segment(0, N) += footPosU*X_.segment(2*N, M);
    transformedX_.segment(N, N) += footPosU*X_.segment(2*N + M, M);

    transformedX_.segment(0, N) += footPosS*feetSupervisor_.getSupportFootStateX()(0);
    transformedX_.segment(N, N) += footPosS*feetSupervisor_.getSupportFootStateY()(0);

    transformedX_.segment(0, N) -= dynCopX.S*getComStateX() + dynCopX.K;
    transformedX_.segment(N, N) -= dynCopY.S*getComStateY() + dynCopY.K;

    VectorX tmp = transformedX_;

    transformedX_.segment(0, N) = dynCopX.Uinv*tmp.segment(0, N);
    transformedX_.segment(N, N) = dynCopY.Uinv*tmp.segment(N, N);
  }

  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(HumanoidWalkgen);
}
