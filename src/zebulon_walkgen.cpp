#include "zebulon_walkgen.h"
#include <cmath>

using namespace MPCWalkgen;

ZebulonWalkgen::ZebulonWalkgen()
:velTrackingObj_(baseModel_)
,posTrackingObj_(baseModel_)
,jerkMinObj_(lipModel_, baseModel_)
,tiltMinObj_(lipModel_, baseModel_)
,copCenteringObj_(lipModel_, baseModel_)
,comCenteringObj_(lipModel_, baseModel_)
,copConstraint_(lipModel_, baseModel_)
,comConstraint_(lipModel_, baseModel_)
,baseMotionConstraint_(baseModel_)
,qpoasesSolver_(makeQPSolver<Scalar>(1, 1))
,invObjNormFactor_(1.0)
,invCtrNormFactor_(1.0)
{
  dX_.setZero(4*lipModel_.getNbSamples());
  X_.setZero(4*lipModel_.getNbSamples());

  config_.withCopConstraints        = true;
  config_.withComConstraints        = true;
  config_.withBaseMotionConstraints = true;

  weighting_.velocityTracking = 0.0;
  weighting_.positionTracking = 0.0;
  weighting_.copCentering     = 0.0;
  weighting_.jerkMinimization = 0.0;

  computeConstantPart();
}

ZebulonWalkgen::~ZebulonWalkgen(){}

void ZebulonWalkgen::setNbSamples(int nbSamples)
{
  assert(nbSamples>0);

  lipModel_.setNbSamples(nbSamples);
  baseModel_.setNbSamples(nbSamples);
  comCenteringObj_.setNbSamples(nbSamples);

  dX_.setZero(4*nbSamples);
  X_.setZero(4*nbSamples);

  jerkMinObj_.computeConstantPart();
  tiltMinObj_.computeConstantPart();
  copConstraint_.computeConstantPart();
  comConstraint_.computeConstantPart();
  copCenteringObj_.computeConstantPart();
  comCenteringObj_.computeConstantPart();
  velTrackingObj_.computeConstantPart();
  posTrackingObj_.computeConstantPart();
  baseMotionConstraint_.computeConstantPart();

  computeConstantPart();
}

void ZebulonWalkgen::setSamplingPeriod(Scalar samplingPeriod)
{
  assert(samplingPeriod>0.0);

  lipModel_.setSamplingPeriod(samplingPeriod);
  baseModel_.setSamplingPeriod(samplingPeriod);

  copConstraint_.computeConstantPart();
  comConstraint_.computeConstantPart();
  copCenteringObj_.computeConstantPart();
  comCenteringObj_.computeConstantPart();
  velTrackingObj_.computeConstantPart();
  posTrackingObj_.computeConstantPart();
  baseMotionConstraint_.computeConstantPart();
  tiltMinObj_.computeConstantPart();

  computeConstantPart();
}

void ZebulonWalkgen::setGravity(const Vector3& gravity)
{
  assert(std::abs(gravity(2))>EPSILON);

  lipModel_.setGravity(gravity);
  baseModel_.setGravity(gravity);

  copConstraint_.computeConstantPart();
  copCenteringObj_.computeConstantPart();
  comCenteringObj_.updateGravityShift();

  computeConstantPart();
}

void ZebulonWalkgen::setBaseCopConvexPolygon(const ConvexPolygon& convexPolygon)
{
  assert(convexPolygon.getNbVertices()>=3);

  baseModel_.setCopSupportConvexPolygon(convexPolygon);

  copConstraint_.computeConstantPart();

  computeConstantPart();
}

void ZebulonWalkgen::setBaseComConvexPolygon(const ConvexPolygon& convexPolygon)
{
  assert(convexPolygon.getNbVertices()>=3);

  baseModel_.setComSupportConvexPolygon(convexPolygon);

  comConstraint_.computeConstantPart();

  computeConstantPart();
}

void ZebulonWalkgen::setComBodyHeight(Scalar comHeight)
{
  assert(comHeight==comHeight);

  lipModel_.setComHeight(comHeight);

  copConstraint_.computeConstantPart();
  copCenteringObj_.computeConstantPart();
  comCenteringObj_.updateGravityShift();
  tiltMinObj_.computeConstantPart();

  computeConstantPart();
}

void ZebulonWalkgen::setComBaseHeight(Scalar comHeight)
{
  assert(comHeight==comHeight);

  baseModel_.setComHeight(comHeight);

  copConstraint_.computeConstantPart();
  copCenteringObj_.computeConstantPart();
  comCenteringObj_.updateGravityShift();

  computeConstantPart();
}

void ZebulonWalkgen::setBodyMass(Scalar mass)
{
  assert(mass==mass);


  Scalar totalMass = mass + baseModel_.getMass();
  lipModel_.setTotalMass(totalMass);
  baseModel_.setTotalMass(totalMass);

  lipModel_.setMass(mass);

  copConstraint_.computeConstantPart();
  copCenteringObj_.computeConstantPart();
  comCenteringObj_.updateGravityShift();
  tiltMinObj_.computeConstantPart();

  computeConstantPart();
}

void ZebulonWalkgen::setBaseMass(Scalar mass)
{
  assert(mass==mass);

  Scalar totalMass = lipModel_.getMass() + mass;
  lipModel_.setTotalMass(totalMass);
  baseModel_.setTotalMass(totalMass);

  baseModel_.setMass(mass);

  copConstraint_.computeConstantPart();
  copCenteringObj_.computeConstantPart();
  comCenteringObj_.updateGravityShift();
  tiltMinObj_.computeConstantPart();

  computeConstantPart();
}


void ZebulonWalkgen::setWheelToBaseDistance(Scalar dist)
{
  baseModel_.setWheelToBaseDistance(dist);

  tiltMinObj_.computeConstantPart();
  computeConstantPart();
}

void ZebulonWalkgen::setAngleWheelToBaseCom(Scalar angle)
{
  baseModel_.setAngleWheelToBaseCom(angle);

  tiltMinObj_.computeConstantPart();
  computeConstantPart();
}

void ZebulonWalkgen::setVelRefInWorldFrame(const VectorX& velRef)
{
  assert(velRef==velRef);
  assert(velRef.size()==baseModel_.getNbSamples()*2);
  velTrackingObj_.setVelRefInWorldFrame(velRef);
}

void ZebulonWalkgen::setPosRefInWorldFrame(const VectorX& posRef)
{
  assert(posRef==posRef);
  assert(posRef.size()==baseModel_.getNbSamples()*2);
  posTrackingObj_.setPosRefInWorldFrame(posRef);
}

void ZebulonWalkgen::setCopRefInLocalFrame(const VectorX& copRef)
{
  assert(copRef==copRef);
  assert(copRef.size()==lipModel_.getNbSamples()*2);
  copCenteringObj_.setCopRefInLocalFrame(copRef);
}

void ZebulonWalkgen::setComRefInLocalFrame(const VectorX& comRef)
{
  assert(comRef==comRef);
  assert(comRef.size()==lipModel_.getNbSamples()*2);
  comCenteringObj_.setComRefInLocalFrame(comRef);
}

void ZebulonWalkgen::setBaseVelLimit(Scalar limit)
{
  assert(limit>=0);

  baseModel_.setVelocityLimit(limit);
}

void ZebulonWalkgen::setBaseAccLimit(Scalar limit)
{
  assert(limit>=0);

  baseModel_.setAccelerationLimit(limit);
}

void ZebulonWalkgen::setBaseJerkLimit(Scalar limit)
{
  assert(limit>=0);

  baseModel_.setJerkLimit(limit);
}

void ZebulonWalkgen::setBaseStateX(const VectorX& state)
{
  assert(state==state);
  assert(state.size()==3);

  baseModel_.setStateX(state);
}

void ZebulonWalkgen::setBaseStateY(const VectorX& state)
{
  assert(state==state);
  assert(state.size()==3);

  baseModel_.setStateY(state);
}

void ZebulonWalkgen::setBaseStateRoll(const VectorX& state)
{
  assert(state==state);
  assert(state.size()==3);

  baseModel_.setStateRoll(state);
}

void ZebulonWalkgen::setBaseStatePitch(const VectorX& state)
{
  assert(state==state);
  assert(state.size()==3);

  baseModel_.setStatePitch(state);
}

void ZebulonWalkgen::setComStateX(const VectorX& state)
{
  assert(state==state);
  assert(state.size()==3);

  lipModel_.setStateX(state);
}

void ZebulonWalkgen::setComStateY(const VectorX& state)
{
  assert(state==state);
  assert(state.size()==3);

  lipModel_.setStateY(state);
}

void ZebulonWalkgen::setWeightings(const  Weighting& weighting)
{
  assert(weighting.copCentering>=0);
  assert(weighting.comCentering>=0);
  assert(weighting.velocityTracking>=0);
  assert(weighting.velocityTracking>=0);
  assert(weighting.jerkMinimization>=0);
  assert(weighting.tiltMinimization>=0);

  weighting_ = weighting;

  computeConstantPart();
}

void ZebulonWalkgen::setConfig(const Config& config)
{
  assert(config.withBaseMotionConstraints == config.withBaseMotionConstraints);
  assert(config.withComConstraints == config.withComConstraints);
  assert(config.withCopConstraints == config.withCopConstraints);

  config_ = config;

  computeConstantPart();
}


bool ZebulonWalkgen::solve(Scalar feedBackPeriod)
{
  int N = lipModel_.getNbSamples();
  int M1 = config_.withCopConstraints? copConstraint_.getNbConstraints() : 0;
  int M2 = config_.withBaseMotionConstraints? baseMotionConstraint_.getNbConstraints() : 0;
  int M3 = config_.withComConstraints? comConstraint_.getNbConstraints() : 0;

  B_ = X_.segment(2*N, 2*N);

  assert(velTrackingObj_.getGradient(B_).size() == 2*N);
  assert(posTrackingObj_.getGradient(B_).size() == 2*N);

  assert(copCenteringObj_.getGradient(X_).size() == 4*N);
  assert(comCenteringObj_.getGradient(X_).size() == 4*N);
  assert(jerkMinObj_.getGradient(X_).size() == 4*N);
  assert(tiltMinObj_.getGradient(X_).size() == 4*N);

  if (config_.withCopConstraints)
  {
    assert(copConstraint_.getFunction(X_).size() == M1);
  }
  if (config_.withComConstraints)
  {
    assert(comConstraint_.getFunction(X_).size() == M3);
  }
  if (config_.withBaseMotionConstraints)
  {
    assert(baseMotionConstraint_.getFunctionInf(B_).size() == M2);
    assert(baseMotionConstraint_.getFunctionSup(B_).size() == M2);
  }
  assert(feedBackPeriod>0);

  qpMatrix_.p.fill(0.0);
  qpMatrix_.bu.fill(10e10);
  qpMatrix_.bl.fill(-10e10);
  qpMatrix_.xu.fill(10e10);
  qpMatrix_.xl.fill(-10e10);

  if (weighting_.velocityTracking>0.0)
  {
    qpMatrix_.p.segment(2*N, 2*N) +=
       weighting_.velocityTracking*velTrackingObj_.getGradient(B_);
  }
  if (weighting_.positionTracking>0.0)
  {
    qpMatrix_.p.segment(2*N, 2*N) +=
        weighting_.positionTracking*posTrackingObj_.getGradient(B_);
  }
  if (weighting_.copCentering>0.0)
  {
    qpMatrix_.p += weighting_.copCentering*copCenteringObj_.getGradient(X_);
  }
  if (weighting_.comCentering>0.0)
  {
    qpMatrix_.p += weighting_.comCentering*comCenteringObj_.getGradient(X_);
  }
  if (weighting_.jerkMinimization>0.0)
  {
    qpMatrix_.p += weighting_.jerkMinimization*jerkMinObj_.getGradient(X_);
  }
  if (weighting_.tiltMinimization>0.0)
  {
    qpMatrix_.p += weighting_.tiltMinimization*tiltMinObj_.getGradient(X_);
  }

  if (config_.withCopConstraints)
  {
    qpMatrix_.bl.segment(0, M1) = copConstraint_.getFunction(X_);
  }
  if (config_.withBaseMotionConstraints)
  {
    qpMatrix_.bl.segment(M1, M2) = baseMotionConstraint_.getFunctionInf(B_);
    qpMatrix_.bu.segment(M1, M2) = baseMotionConstraint_.getFunctionSup(B_);

    qpMatrix_.xu.segment(2*N, 2*N).fill(baseModel_.getJerkLimit());
    qpMatrix_.xu.segment(2*N, 2*N) -= B_;

    qpMatrix_.xl.segment(2*N, 2*N).fill(-baseModel_.getJerkLimit());
    qpMatrix_.xl.segment(2*N, 2*N) -= B_;
  }
  if (config_.withComConstraints)
  {
    qpMatrix_.bl.segment(M1+M2, M3) = comConstraint_.getFunction(X_);
  }

  qpMatrix_.p  *= invObjNormFactor_;
  qpMatrix_.bu *= invCtrNormFactor_;
  qpMatrix_.bl *= invCtrNormFactor_;

  bool solutionFound = qpoasesSolver_->solve(qpMatrix_, dX_, true);
  X_ += dX_;

  lipModel_.updateStateX(X_(0), feedBackPeriod);
  lipModel_.updateStateY(X_(N), feedBackPeriod);
  baseModel_.updateStateX(X_(2*N), feedBackPeriod);
  baseModel_.updateStateY(X_(3*N), feedBackPeriod);

  return solutionFound;
}


const VectorX& ZebulonWalkgen::getBaseStateX()
{
  return baseModel_.getStateX();
}

const VectorX& ZebulonWalkgen::getBaseStateY()
{
  return baseModel_.getStateY();
}

const VectorX& ZebulonWalkgen::getComStateX()
{
  return lipModel_.getStateX();
}

const VectorX& ZebulonWalkgen::getComStateY()
{
  return lipModel_.getStateY();
}

void ZebulonWalkgen::computeConstantPart()
{
  int N = lipModel_.getNbSamples();
  int M1 = config_.withCopConstraints? copConstraint_.getNbConstraints() : 0;
  int M2 = config_.withBaseMotionConstraints? baseMotionConstraint_.getNbConstraints() : 0;
  int M3 = config_.withComConstraints? comConstraint_.getNbConstraints() : 0;
  int M = M1+M2+M3;

  assert(velTrackingObj_.getHessian().rows() == 2*N);
  assert(velTrackingObj_.getHessian().cols() == 2*N);

  assert(posTrackingObj_.getHessian().rows() == 2*N);
  assert(posTrackingObj_.getHessian().cols() == 2*N);

  assert(copCenteringObj_.getHessian().rows() == 4*N);
  assert(copCenteringObj_.getHessian().cols() == 4*N);

  assert(comCenteringObj_.getHessian().rows() == 4*N);
  assert(comCenteringObj_.getHessian().cols() == 4*N);

  assert(jerkMinObj_.getHessian().rows() == 4*N);
  assert(jerkMinObj_.getHessian().cols() == 4*N);

  assert(tiltMinObj_.getHessian().rows() == 4*N);
  assert(tiltMinObj_.getHessian().cols() == 4*N);

  if (config_.withCopConstraints)
  {
    assert(copConstraint_.getGradient().cols() == 4*N);
    assert(copConstraint_.getGradient().rows() == M1);
  }
  if (config_.withComConstraints)
  {
    assert(comConstraint_.getGradient().cols() == 4*N);
    assert(comConstraint_.getGradient().rows() == M3);
  }
  if (config_.withBaseMotionConstraints)
  {
    assert(baseMotionConstraint_.getGradient().cols() == 2*N);
    assert(baseMotionConstraint_.getGradient().rows() == M2);
  }

  qpoasesSolver_.reset(makeQPSolver<Scalar>(4*N, M));

  qpMatrix_.Q.setZero(4*N, 4*N);
  qpMatrix_.p.setZero(4*N, 1);
  qpMatrix_.A.setZero(M, 4*N);
  qpMatrix_.bl.setZero(M, 1);
  qpMatrix_.bu.setZero(M, 1);
  qpMatrix_.xl.setZero(4*N, 1);
  qpMatrix_.xu.setZero(4*N, 1);

  if (weighting_.velocityTracking>0.0)
  {
    qpMatrix_.Q.block(2*N, 2*N, 2*N, 2*N) +=
       weighting_.velocityTracking*velTrackingObj_.getHessian();
  }
  if (weighting_.positionTracking>0.0)
  {
    qpMatrix_.Q.block(2*N, 2*N, 2*N, 2*N) +=
       weighting_.positionTracking*posTrackingObj_.getHessian();
  }
  if (weighting_.copCentering>0.0)
  {
    qpMatrix_.Q += weighting_.copCentering*copCenteringObj_.getHessian();
  }
  if (weighting_.comCentering>0.0)
  {
    qpMatrix_.Q += weighting_.comCentering*comCenteringObj_.getHessian();
  }
  if (weighting_.jerkMinimization>0.0)
  {
    qpMatrix_.Q += weighting_.jerkMinimization*jerkMinObj_.getHessian();
  }
  if (weighting_.tiltMinimization>0.0)
  {
    qpMatrix_.Q += weighting_.tiltMinimization*tiltMinObj_.getHessian();
  }

  if (config_.withCopConstraints)
  {
    qpMatrix_.A.block(0, 0, M1, 4*N) = copConstraint_.getGradient();
  }
  if (config_.withBaseMotionConstraints)
  {
    qpMatrix_.A.block(M1, 2*N, M2, 2*N) = baseMotionConstraint_.getGradient();
  }
  if (config_.withComConstraints)
  {
    qpMatrix_.A.block(M1+M2, 0, M3, 4*N) = comConstraint_.getGradient();
  }

  computeNormalizationFactor(qpMatrix_.Q, qpMatrix_.A);
  qpMatrix_.Q *= invObjNormFactor_;
  qpMatrix_.A *= invCtrNormFactor_;

  qpMatrix_.At = qpMatrix_.A.transpose();

}

void ZebulonWalkgen::computeNormalizationFactor(MatrixX& Q, MatrixX& A)
{
  int Qr = Q.rows();
  int Qc = Q.cols();
  invObjNormFactor_=1.0;
  for(int i=0; i<Qr; ++i)
  {
    for(int j=0; j<Qc; ++j)
    {
      Scalar v = std::abs(Q(i, j));
      if (v>EPSILON && v<invObjNormFactor_)
      {
        invObjNormFactor_ = v;
      }
    }
  }
  invObjNormFactor_ = 1.0/invObjNormFactor_;

  int Ar = A.rows();
  int Ac = A.cols();
  invCtrNormFactor_=1.0;
  for(int i=0; i<Ar; ++i)
  {
    for(int j=0; j<Ac; ++j)
    {
      Scalar v = std::abs(A(i, j));
      if (v>EPSILON && v<invCtrNormFactor_)
      {
        invCtrNormFactor_ = v;
      }
    }
  }
  invCtrNormFactor_ = 1.0/invCtrNormFactor_;
}


