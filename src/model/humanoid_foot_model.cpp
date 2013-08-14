#include "humanoid_foot_model.h"

namespace MPCWalkgen
{

  HumanoidFootModel::KinematicLimits::KinematicLimits()
    :hipYawUpperBound_(0)
    ,hipYawSpeedUpperBound_(0)
    ,hipYawAccelerationUpperBound_(0)
    ,hipYawLowerBound_(0)
    ,maxHeight_(0)
  {}

  HumanoidFootModel::HumanoidFootModel(int nbSamples,
                                       Scalar samplingPeriod,
                                       int nbPreviewedSteps)
    :nbSamples_(nbSamples)
    ,samplingPeriod_(samplingPeriod)
    ,nbPreviewedSteps_(nbPreviewedSteps)
    ,kinematicLimits_()
  {

    assert(samplingPeriod>0);
    assert(nbSamples>0);

    //For now we only need to know the foot position, so the state vector size is 1
    stateX_.setZero(1);
    stateY_.setZero(1);
    stateZ_.setZero(1);
    stateYaw_.setZero(1);

    computeSelectionMatrix();
    computeFootPosDynamic();
    computeRotationMatrix();

  }

  HumanoidFootModel::HumanoidFootModel()
    :nbSamples_(1)
    ,samplingPeriod_(1.0)
    ,nbPreviewedSteps_(1)
    ,kinematicLimits_()
  {
    stateX_.setZero(1);
    stateY_.setZero(1);
    stateZ_.setZero(1);
    stateYaw_.setZero(1);

    computeSelectionMatrix();
    computeFootPosDynamic();
    computeRotationMatrix();
  }

  HumanoidFootModel::~HumanoidFootModel(){}

  void HumanoidFootModel::setNbSamples(int nbSamples)
  {
    assert(nbSamples>0);

    nbSamples_ = nbSamples;

    computeSelectionMatrix();
    computeFootPosDynamic();
    computeRotationMatrix();
  }

  void HumanoidFootModel::setSamplingPeriod(Scalar samplingPeriod)
  {
    assert(samplingPeriod>0);

    samplingPeriod_ = samplingPeriod;

    computeSelectionMatrix();
    computeFootPosDynamic();
    computeRotationMatrix();
  }

  void HumanoidFootModel::computeSelectionMatrix()
  {
    selectionMatrix_.reset(getNbSamples(), getNbPreviewedSteps());
    //TODO: complete
  }

  void HumanoidFootModel::computeRotationMatrix()
  {
    rotationMatrix_.setZero(2*getNbSamples(), 2*getNbSamples());
    rotationMatrixT_.setZero(2*getNbSamples(), 2*getNbSamples());
    //TODO: complete. WARNING: rotation of -yaw
  }

  void HumanoidFootModel::computeFootPosDynamic()
  {
    footPosDynamic_ = selectionMatrix_.toLinearDynamics();
  }

  void HumanoidFootModel::setHipYawUpperBound(
      Scalar hipYawUpperBound)
  {kinematicLimits_.hipYawUpperBound_ = hipYawUpperBound;}

  void HumanoidFootModel::setHipYawSpeedUpperBound(
      Scalar hipYawSpeedUpperBound)
  {kinematicLimits_.hipYawSpeedUpperBound_ = hipYawSpeedUpperBound;}

  void HumanoidFootModel::setHipYawAccelerationUpperBound(
      Scalar hipYawAccelerationUpperBound)
  {kinematicLimits_.hipYawAccelerationUpperBound_ = hipYawAccelerationUpperBound;}

  void HumanoidFootModel::setHipYawLowerBound(
      Scalar hipYawLowerBound)
  {kinematicLimits_.hipYawLowerBound_ = hipYawLowerBound;}

  void HumanoidFootModel::setMaxHeight(
      Scalar maxHeight)
  {kinematicLimits_.maxHeight_ = maxHeight;}

  void HumanoidFootModel::SelectionMatrix::reset(int nbSamples,
                                                 int nbPreviewedSteps)
  {
    V.setZero(nbSamples, nbPreviewedSteps);
    VT.setZero(nbPreviewedSteps, nbSamples);
    V0.setZero(nbSamples, 1);
    V0T.setZero(1, nbSamples);
  }

  //Change this if new the state vector is increased with speed, acceleration..
  //If so, selection matrices will have to be multiplied with dynamic terms
  LinearDynamic HumanoidFootModel::SelectionMatrix::toLinearDynamics()
  {
    LinearDynamic output;

    output.reset(V0.rows(), V0.cols(), V.cols());

    //TODO: check if this cast works
    output.U = V.cast<Scalar>();
    output.UT = VT.cast<Scalar>();
    output.S = V0.cast<Scalar>();
    output.ST = V0T.cast<Scalar>();

    return output;
  }

}
