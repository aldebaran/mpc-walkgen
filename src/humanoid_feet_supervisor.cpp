////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_feet_supervisor.h
///\brief Implement the supervisor that manage the FSM
///\author de Gourcuff Martin
///\date 22/08/13
///
////////////////////////////////////////////////////////////////////////////////

#include "humanoid_feet_supervisor.h"

namespace MPCWalkgen
{

  void HumanoidFeetSupervisor::SelectionMatrices::reset(
      unsigned int nbSamples,
      unsigned int nbPreviewedSteps)
  {
    assert(nbSamples>0);
    assert(nbPreviewedSteps>=0);

    V.setZero(nbSamples, nbPreviewedSteps);
    VT.setZero(nbPreviewedSteps, nbSamples);
    V0.setZero(nbSamples, 1);
    V0T.setZero(1, nbSamples);
  }

  LinearDynamic HumanoidFeetSupervisor::SelectionMatrices::toLinearDynamics()
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


  HumanoidFeetSupervisor::HumanoidFeetSupervisor(const HumanoidFootModel& leftFoot,
                                                 const HumanoidFootModel& rightFoot,
                                                 unsigned int nbSamples,
                                                 Scalar samplingPeriod)
    :leftFootModel_(leftFoot)
    ,rightFootModel_(rightFoot)
    ,nbSamples_(nbSamples)
    ,samplingPeriod_(samplingPeriod)
    ,nbPreviewedSteps_(0)
    ,copConvexPolygonVec_()
    ,sampleID_(0)
  {
    //TODO: Complete

    xComputeSelectionMatrix();
    xComputeFeetPosDynamic();
    xComputeRotationMatrix();
  }

  HumanoidFeetSupervisor::HumanoidFeetSupervisor(const HumanoidFootModel& leftFoot,
                                                 const HumanoidFootModel& rightFoot)
    :leftFootModel_(leftFoot)
    ,rightFootModel_(rightFoot)
    ,nbSamples_(1)
    ,samplingPeriod_(1.0)
    ,nbPreviewedSteps_(0)
    ,copConvexPolygonVec_()
    ,sampleID_(0)
  {
    //TODO: Complete

    xComputeSelectionMatrix();
    xComputeFeetPosDynamic();
    xComputeRotationMatrix();
  }

  HumanoidFeetSupervisor::~HumanoidFeetSupervisor()
  {}

  void HumanoidFeetSupervisor::setNbSamples(int nbSamples)
  {
    assert(nbSamples>0);

    leftFootModel_.setNbSamples(nbSamples);
    rightFootModel_.setNbSamples(nbSamples);

    computeConstantPart();
  }

  void HumanoidFeetSupervisor::setSamplingPeriod(Scalar samplingPeriod)
  {
    assert(samplingPeriod>=0);

    leftFootModel_.setSamplingPeriod(samplingPeriod);
    rightFootModel_.setSamplingPeriod(samplingPeriod);

    computeConstantPart();
  }

  void HumanoidFeetSupervisor::setStepPeriod(Scalar stepPeriod)
  {
    //TODO: complete
  }

  void HumanoidFeetSupervisor::setLeftFootKinematicConvexPolygon(
      const ConvexPolygon& convexPolygon)
  {
    leftFootModel_.setKinematicConvexPolygon(convexPolygon);
  }

  void HumanoidFeetSupervisor::setRightFootKinematicConvexPolygon(
      const ConvexPolygon &convexPolygon)
  {
    rightFootModel_.setKinematicConvexPolygon(convexPolygon);
  }

  void HumanoidFeetSupervisor::setLeftFootCopConvexPolygon(
      const ConvexPolygon& convexPolygon)
  {
    leftFootModel_.setCopConvexPolygon(convexPolygon);
  }

  void HumanoidFeetSupervisor::setRightFootCopConvexPolygon(
      const ConvexPolygon& convexPolygon)
  {
    leftFootModel_.setCopConvexPolygon(convexPolygon);
  }

  void HumanoidFeetSupervisor::setLeftFootStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    leftFootModel_.setStateX(state);
  }

  void HumanoidFeetSupervisor::setLeftFootStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    leftFootModel_.setStateY(state);
  }

  void HumanoidFeetSupervisor::setLeftFootStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    leftFootModel_.setStateZ(state);
  }

  void HumanoidFeetSupervisor::setRightFootStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    rightFootModel_.setStateX(state);
  }

  void HumanoidFeetSupervisor::setRightFootStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    rightFootModel_.setStateY(state);
  }

  void HumanoidFeetSupervisor::setRightFootStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    rightFootModel_.setStateZ(state);
  }

  void HumanoidFeetSupervisor::setLeftFootMaxHeight(
      Scalar leftFootMaxHeight)
  {
    leftFootModel_.setMaxHeight(
          leftFootMaxHeight);
  }

  void HumanoidFeetSupervisor::setRightFootMaxHeight(
      Scalar rightFootMaxHeight)
  {
    rightFootModel_.setMaxHeight(
          rightFootMaxHeight);
  }

  void HumanoidFeetSupervisor::setLeftFootYawUpperBound(
      Scalar leftFootYawUpperBound)
  {
    leftFootModel_.setHipYawUpperBound(
          leftFootYawUpperBound);
  }
  void HumanoidFeetSupervisor::setLeftFootYawLowerBound(
      Scalar leftFootYawLowerBound)
  {
    leftFootModel_.setHipYawLowerBound(
          leftFootYawLowerBound);
  }
  void HumanoidFeetSupervisor::setRightFootYawUpperBound(
      Scalar rightFootYawUpperBound)
  {
    rightFootModel_.setHipYawUpperBound(
          rightFootYawUpperBound);
  }
  void HumanoidFeetSupervisor::setRightFootYawLowerBound(
      Scalar rightFootYawLowerBound)
  {
    rightFootModel_.setHipYawLowerBound(
          rightFootYawLowerBound);
  }

  void HumanoidFeetSupervisor::setLeftFootYawSpeedUpperBound(
      Scalar leftFootYawSpeedUpperBound)
  {
    leftFootModel_.setHipYawSpeedUpperBound(
          leftFootYawSpeedUpperBound);
  }
  void HumanoidFeetSupervisor::setRightFootYawSpeedUpperBound(
      Scalar rightFootYawSpeedUpperBound)
  {
    rightFootModel_.setHipYawSpeedUpperBound(
          rightFootYawSpeedUpperBound);
  }

  void HumanoidFeetSupervisor::setLeftFootYawAccelerationUpperBound(
      Scalar leftFootYawAccelerationUpperBound)
  {
    leftFootModel_.setHipYawAccelerationUpperBound(
          leftFootYawAccelerationUpperBound);
  }
  void HumanoidFeetSupervisor::setRightFootYawAccelerationUpperBound(
      Scalar rightFootYawAccelerationUpperBound)
  {
    rightFootModel_.setHipYawAccelerationUpperBound(
          rightFootYawAccelerationUpperBound);
  }


  unsigned int HumanoidFeetSupervisor::sampleToStep(unsigned int sampleNb) const
  {
    //TODO: Complete
    return 0;
  }
  void HumanoidFeetSupervisor::computeConstantPart()
  {
    //TODO: Complete
  }

  void HumanoidFeetSupervisor::xComputeSelectionMatrix()
  {
    selectionMatrices_.reset(getNbSamples(), getNbPreviewedSteps());
    //TODO: complete
  }

  void HumanoidFeetSupervisor::xComputeFeetPosDynamic()
  {
    feetPosDynamic_ = selectionMatrices_.toLinearDynamics();
  }

  void HumanoidFeetSupervisor::xComputeRotationMatrix()
  {
    rotationMatrix_.setZero(2*getNbSamples(), 2*getNbSamples());
    rotationMatrixT_.setZero(2*getNbSamples(), 2*getNbSamples());
    //TODO: complete. WARNING: rotation of -yaw
  }
}
