////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/zebulon_walkgen.h>
#include <iostream>
#include <mpc-walkgen/constant.h>
#include <cmath>
#include "macro.h"

namespace MPCWalkgen
{

template <typename Scalar>
ZebulonWalkgen<Scalar>::ZebulonWalkgen()
:velTrackingObj_(baseModel_)
,posTrackingObj_(baseModel_)
,jerkMinObj_(lipModel_, baseModel_)
,tiltMinObj_(lipModel_, baseModel_)
,tiltVelMinObj_(lipModel_, baseModel_)
,copCenteringObj_(lipModel_, baseModel_)
,comCenteringObj_(lipModel_, baseModel_)
,copConstraint_(lipModel_, baseModel_)
,comConstraint_(lipModel_, baseModel_)
,baseMotionConstraint_(baseModel_)
,tiltMotionConstraint_(lipModel_, baseModel_)
,qpoasesSolver_(makeQPSolver<Scalar>(1, 1))
,invObjNormFactor_(1.0)
,invCtrNormFactor_(1.0)
{
  dX_.setZero(4*lipModel_.getNbSamples());
  X_.setZero(4*lipModel_.getNbSamples());

  computeConstantPart();
}

template <typename Scalar>
ZebulonWalkgen<Scalar>::~ZebulonWalkgen(){}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setNbSamples(int nbSamples)
{
  assert(nbSamples>0);

  lipModel_.setNbSamples(nbSamples);
  baseModel_.setNbSamples(nbSamples);
  comCenteringObj_.setNbSamples(nbSamples);

  dX_.setZero(4*nbSamples);
  X_.setZero(4*nbSamples);

  jerkMinObj_.computeConstantPart();
  tiltMinObj_.computeConstantPart();
  tiltVelMinObj_.computeConstantPart();
  copConstraint_.computeConstantPart();
  comConstraint_.computeConstantPart();
  copCenteringObj_.computeConstantPart();
  comCenteringObj_.computeConstantPart();
  velTrackingObj_.computeConstantPart();
  posTrackingObj_.computeConstantPart();
  baseMotionConstraint_.computeConstantPart();
  tiltMotionConstraint_.computeConstantPart();

  computeConstantPart();
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setSamplingPeriod(Scalar samplingPeriod)
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
  tiltMotionConstraint_.computeConstantPart();
  tiltMinObj_.computeConstantPart();
  tiltVelMinObj_.computeConstantPart();

  computeConstantPart();
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setGravity(const Vector3& gravity)
{
  assert(std::abs(gravity(2))>Constant<Scalar>::EPSILON);

  lipModel_.setGravity(gravity);
  baseModel_.setGravity(gravity);

  copConstraint_.computeConstantPart();
  copCenteringObj_.computeConstantPart();
  comCenteringObj_.updateGravityShift();

  computeConstantPart();
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setBaseCopConvexPolygon(const ConvexPolygon<Scalar>& convexPolygon)
{
  assert(convexPolygon.getNbVertices()>=3);

  baseModel_.setCopSupportConvexPolygon(convexPolygon);

  copConstraint_.computeConstantPart();

  computeConstantPart();
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setBaseComConvexPolygon(const ConvexPolygon<Scalar>& convexPolygon)
{
  assert(convexPolygon.getNbVertices()>=3);

  baseModel_.setComSupportConvexPolygon(convexPolygon);

  comConstraint_.computeConstantPart();

  computeConstantPart();
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setBaseCopHull(const vectorOfVector3 &p)
{
  //Ugly but necessary to keep compatibility with Naoqi
  vectorOfVector2 p2D(p.size());

  for(size_t i=0; i<p.size(); ++i)
  {
    p2D[i] = p[i].template head<2>();
  }
  setBaseCopConvexPolygon(ConvexPolygon<Scalar>(p2D));
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setBaseComHull(const vectorOfVector3& p)
{
  vectorOfVector2 p2D(p.size());
  for(size_t i=0; i<p.size(); ++i)
  {
    p2D[i] = p[i].template head<2>();
  }
  setBaseComConvexPolygon(ConvexPolygon<Scalar>(p2D));
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setComBodyHeight(Scalar comHeight)
{
  assert(comHeight==comHeight);

  lipModel_.setComHeight(comHeight);

  copConstraint_.computeConstantPart();
  copCenteringObj_.computeConstantPart();
  comCenteringObj_.updateGravityShift();
  tiltMinObj_.computeConstantPart();
  tiltVelMinObj_.computeConstantPart();

  computeConstantPart();
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setComBaseHeight(Scalar comHeight)
{
  assert(comHeight==comHeight);

  baseModel_.setComHeight(comHeight);

  copConstraint_.computeConstantPart();
  copCenteringObj_.computeConstantPart();
  comCenteringObj_.updateGravityShift();

  computeConstantPart();
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setBodyMass(Scalar mass)
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
  tiltVelMinObj_.computeConstantPart();

  computeConstantPart();
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setBaseMass(Scalar mass)
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
  tiltVelMinObj_.computeConstantPart();

  computeConstantPart();
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setVelRefInWorldFrame(const VectorX& velRef)
{
  assert(velRef==velRef);
  assert(velRef.size()==baseModel_.getNbSamples()*2);
  velTrackingObj_.setVelRefInWorldFrame(velRef);
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setPosRefInWorldFrame(const VectorX& posRef)
{
  assert(posRef==posRef);
  assert(posRef.size()==baseModel_.getNbSamples()*2);
  posTrackingObj_.setPosRefInWorldFrame(posRef);
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setCopRefInLocalFrame(const VectorX& copRef)
{
  assert(copRef==copRef);
  assert(copRef.size()==lipModel_.getNbSamples()*2);
  copCenteringObj_.setCopRefInLocalFrame(copRef);
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setComRefInLocalFrame(const VectorX& comRef)
{
  assert(comRef==comRef);
  assert(comRef.size()==lipModel_.getNbSamples()*2);
  comCenteringObj_.setComRefInLocalFrame(comRef);
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setComAndCopRefInWorldFrame(const VectorX& comPosRefInWorldFrame,
                                                         const VectorX& comAccRefInWorldFrame,
                                                         const VectorX& basePosRefInWorldFrame,
                                                         const VectorX& baseAccRefInWorldFrame)
{
#ifndef NDEBUG
  // Only used in following asserts
  const int refSize = 2 * lipModel_.getNbSamples();
#endif
  assert(comPosRefInWorldFrame==comPosRefInWorldFrame);
  assert(basePosRefInWorldFrame==basePosRefInWorldFrame);
  assert(comPosRefInWorldFrame.size()==refSize);
  assert(basePosRefInWorldFrame.size()==refSize);
  assert(comAccRefInWorldFrame==comAccRefInWorldFrame);
  assert(baseAccRefInWorldFrame==baseAccRefInWorldFrame);
  assert(comAccRefInWorldFrame.size()==refSize);
  assert(baseAccRefInWorldFrame.size()==refSize);

  comCenteringObj_.setComRefInLocalFrame(comPosRefInWorldFrame-basePosRefInWorldFrame);
  copCenteringObj_.setCopRefInWorldFrame(comPosRefInWorldFrame, comAccRefInWorldFrame,
                                         basePosRefInWorldFrame, baseAccRefInWorldFrame);

}




template <typename Scalar>
void ZebulonWalkgen<Scalar>::setBaseVelLimit(Scalar limit)
{
  assert(limit>=0);

  baseModel_.setVelocityLimit(limit);
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setBaseAccLimit(Scalar limit)
{
  assert(limit>=0);

  baseModel_.setAccelerationLimit(limit);
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setBaseJerkLimit(Scalar limit)
{
  assert(limit>=0);

  baseModel_.setJerkLimit(limit);
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setBaseStateX(const VectorX& state)
{
  assert(state==state);
  assert(state.size()==3);

  baseModel_.setStateX(state);
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setBaseStateY(const VectorX& state)
{
  assert(state==state);
  assert(state.size()==3);

  baseModel_.setStateY(state);
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setBaseStateRoll(const VectorX& state)
{
  assert(state==state);
  assert(state.size()==3);

  baseModel_.setStateRoll(state);
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setBaseStatePitch(const VectorX& state)
{
  assert(state==state);
  assert(state.size()==3);

  baseModel_.setStatePitch(state);
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setBaseStateYaw(const VectorX& state)
{
  assert(state==state);
  assert(state.size()==3);

  baseModel_.setStateYaw(state);
  tiltMotionConstraint_.computeConstantPart();
  computeConstantPart();
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setComStateX(const VectorX& state)
{
  assert(state==state);
  assert(state.size()==3);

  lipModel_.setStateX(state);
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setComStateY(const VectorX& state)
{
  assert(state==state);
  assert(state.size()==3);

  lipModel_.setStateY(state);
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setTiltContactPointOnTheGroundInLocalFrameX(Scalar pos)
{
  assert(pos==pos);
  baseModel_.setTiltContactPointX(pos);
  tiltMinObj_.updateTiltContactPoint();
  tiltVelMinObj_.updateTiltContactPoint();
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setTiltContactPointOnTheGroundInLocalFrameY(Scalar pos)
{
  assert(pos==pos);
  baseModel_.setTiltContactPointY(pos);
  tiltMinObj_.updateTiltContactPoint();
  tiltVelMinObj_.updateTiltContactPoint();
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setWeightings(const  ZebulonWalkgenWeighting<Scalar>& weighting)
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

template <typename Scalar>
void ZebulonWalkgen<Scalar>::setConfig(const ZebulonWalkgenConfig<Scalar>& config)
{
  assert(config.withBaseMotionConstraints == config.withBaseMotionConstraints);
  assert(config.withComConstraints == config.withComConstraints);
  assert(config.withCopConstraints == config.withCopConstraints);

  config_ = config;

  computeConstantPart();
}

template <typename Scalar>
bool ZebulonWalkgen<Scalar>::solve(Scalar feedBackPeriod)
{
  int N = lipModel_.getNbSamples();
  int M1 = config_.withCopConstraints? copConstraint_.getNbConstraints() : 0;
  int M2 = config_.withBaseMotionConstraints? baseMotionConstraint_.getNbConstraints() : 0;
  int M3 = config_.withComConstraints? comConstraint_.getNbConstraints() : 0;
  int M4 = config_.withTiltMotionConstraints? tiltMotionConstraint_.getNbConstraints() : 0;

  B_ = X_.segment(2*N, 2*N);

  assert(velTrackingObj_.getGradient(B_).size() == 2*N);
  assert(posTrackingObj_.getGradient(B_).size() == 2*N);

  assert(copCenteringObj_.getGradient(X_).size() == 4*N);
  assert(comCenteringObj_.getGradient(X_).size() == 4*N);
  assert(jerkMinObj_.getGradient(X_).size() == 4*N);
  assert(tiltMinObj_.getGradient(X_).size() == 4*N);
  assert(tiltVelMinObj_.getGradient(X_).size() == 4*N);

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
  if (config_.withTiltMotionConstraints)
  {
    assert(tiltMotionConstraint_.getFunction(X_).size() == M4);
  }

  assert(feedBackPeriod>0);

  qpMatrix_.p.fill(Scalar(0.0));
  qpMatrix_.bu.fill(Scalar(10e10));
  qpMatrix_.bl.fill(Scalar(-10e10));
  qpMatrix_.xu.fill(Scalar(10e10));
  qpMatrix_.xl.fill(Scalar(-10e10));

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
  if (weighting_.tiltVelMinimization>0.0)
  {
    qpMatrix_.p += weighting_.tiltVelMinimization*tiltVelMinObj_.getGradient(X_);
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
  if (config_.withTiltMotionConstraints)
  {
    qpMatrix_.bl.segment(M1+M2+M3, M4) = tiltMotionConstraint_.getFunction(X_);
    qpMatrix_.bu.segment(M1+M2+M3, M4) = tiltMotionConstraint_.getFunction(X_);
  }

  qpMatrix_.p  *= invObjNormFactor_;
  qpMatrix_.bu *= invCtrNormFactor_;
  qpMatrix_.bl *= invCtrNormFactor_;

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
    std::cerr << "m : " << lipModel_.getMass() << std::endl;
    std::cerr << "M : " << baseModel_.getMass() << std::endl;
    std::cerr << "h : " << lipModel_.getComHeight() << std::endl;
    std::cerr << "L : " << baseModel_.getComHeight() << std::endl;
    std::cerr << "T : " << lipModel_.getSamplingPeriod() << std::endl;
    std::cerr << "cx: " << lipModel_.getStateX() << std::endl;
    std::cerr << "bx: " << baseModel_.getStateX() << std::endl;
    std::cerr << "cy: " << lipModel_.getStateY() << std::endl;
    std::cerr << "by: " << baseModel_.getStateY() << std::endl;
    std::cerr << "bY: " << baseModel_.getStateYaw() << std::endl;
    std::cerr << "bP: " << baseModel_.getStatePitch() << std::endl;
    std::cerr << "bR: " << baseModel_.getStateRoll() << std::endl;
  }

  X_ += dX_;


  lipModel_.updateStateX(X_(0), feedBackPeriod);
  lipModel_.updateStateY(X_(N), feedBackPeriod);
  baseModel_.updateStateX(X_(2*N), feedBackPeriod);
  baseModel_.updateStateY(X_(3*N), feedBackPeriod);

  return solutionFound;
}


template <typename Scalar>
const typename Type<Scalar>::VectorX& ZebulonWalkgen<Scalar>::getBaseStateX() const
{
  return baseModel_.getStateX();
}

template <typename Scalar>
const typename Type<Scalar>::VectorX& ZebulonWalkgen<Scalar>::getBaseStateY() const
{
  return baseModel_.getStateY();
}

template <typename Scalar>
const typename Type<Scalar>::VectorX& ZebulonWalkgen<Scalar>::getComStateX() const
{
  return lipModel_.getStateX();
}

template <typename Scalar>
const typename Type<Scalar>::VectorX& ZebulonWalkgen<Scalar>::getComStateY() const
{
  return lipModel_.getStateY();
}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::computeConstantPart()
{
  int N = lipModel_.getNbSamples();
  int M1 = config_.withCopConstraints? copConstraint_.getNbConstraints() : 0;
  int M2 = config_.withBaseMotionConstraints? baseMotionConstraint_.getNbConstraints() : 0;
  int M3 = config_.withComConstraints? comConstraint_.getNbConstraints() : 0;
  int M4 = config_.withTiltMotionConstraints? tiltMotionConstraint_.getNbConstraints() : 0;
  int M = M1+M2+M3+M4;

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

  assert(tiltVelMinObj_.getHessian().rows() == 4*N);
  assert(tiltVelMinObj_.getHessian().cols() == 4*N);

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
  if (config_.withTiltMotionConstraints)
  {
    assert(tiltMotionConstraint_.getGradient().cols() == 4*N);
    assert(tiltMotionConstraint_.getGradient().rows() == M4);
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
  if (weighting_.tiltVelMinimization>0.0)
  {
    qpMatrix_.Q += weighting_.tiltVelMinimization*tiltVelMinObj_.getHessian();
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
  if (config_.withTiltMotionConstraints)
  {
    qpMatrix_.A.block(M1+M2+M3, 0, M4, 4*N) = tiltMotionConstraint_.getGradient();
  }

  computeNormalizationFactor(qpMatrix_.Q, qpMatrix_.A);
  qpMatrix_.Q *= invObjNormFactor_;
  qpMatrix_.A *= invCtrNormFactor_;

  qpMatrix_.At = qpMatrix_.A.transpose();

}

template <typename Scalar>
void ZebulonWalkgen<Scalar>::computeNormalizationFactor(MatrixX& Q, MatrixX& A)
{
  int Qr = Q.rows();
  int Qc = Q.cols();
  invObjNormFactor_ = 1.0f;
  for(int i=0; i<Qr; ++i)
  {
    for(int j=0; j<Qc; ++j)
    {
      Scalar v = std::abs(Q(i, j));
      if (v>Constant<Scalar>::EPSILON && v<invObjNormFactor_)
      {
        invObjNormFactor_ = v;
      }
    }
  }
  invObjNormFactor_ = 1.0f/invObjNormFactor_;

  int Ar = A.rows();
  int Ac = A.cols();
  invCtrNormFactor_ = 1.0f;
  for(int i=0; i<Ar; ++i)
  {
    for(int j=0; j<Ac; ++j)
    {
      Scalar v = std::abs(A(i, j));
      if (v>Constant<Scalar>::EPSILON && v<invCtrNormFactor_)
      {
        invCtrNormFactor_ = v;
      }
    }
  }
  invCtrNormFactor_ = 1.0f/invCtrNormFactor_;
}

  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(ZebulonWalkgen);
}

