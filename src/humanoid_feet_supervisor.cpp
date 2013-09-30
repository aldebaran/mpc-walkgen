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
      int nbSamples,
      int nbPreviewedSteps)
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
                                                 int nbSamples,
                                                 Scalar samplingPeriod)
    :leftFootModel_(leftFoot)
    ,rightFootModel_(rightFoot)
    ,nbSamples_(nbSamples)
    ,samplingPeriod_(samplingPeriod)
    ,nbPreviewedSteps_(0)
    ,stepPeriod_(samplingPeriod) //TODO: add it to constructor argument
    ,copConvexPolygons_(1)
    ,kinematicConvexPolygons_(1)
  {
    init();
  }

  HumanoidFeetSupervisor::HumanoidFeetSupervisor(const HumanoidFootModel& leftFoot,
                                                 const HumanoidFootModel& rightFoot)
    :leftFootModel_(leftFoot)
    ,rightFootModel_(rightFoot)
    ,nbSamples_(1)
    ,samplingPeriod_(1.0)
    ,nbPreviewedSteps_(0)
    ,stepPeriod_(1.0)
    ,copConvexPolygons_(1)
    ,kinematicConvexPolygons_(1)
  {
    init();
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
    assert(stepPeriod>0);
    stepPeriod_ = stepPeriod;
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


  int HumanoidFeetSupervisor::sampleToStep(int sampleNb) const
  {
    //TODO: Complete
    return 0;
  }

  int HumanoidFeetSupervisor::getMaximumNbOfSteps()
  {
    return static_cast<int>(std::floor(nbSamples_*samplingPeriod_/stepPeriod_));
  }

  int HumanoidFeetSupervisor::getMaximumNbOfCopConstraints()
  {
    return std::max(leftFootModel_.getCopConvexPolygon().getNbVertices(),
                    rightFootModel_.getCopConvexPolygon().getNbVertices());
  }

  int HumanoidFeetSupervisor::getMaximumNbOfKinematicConstraints()
  {
    return std::max(leftFootModel_.getKinematicConvexPolygon().getNbVertices(),
                    rightFootModel_.getKinematicConvexPolygon().getNbVertices());
  }

  void HumanoidFeetSupervisor::computeConstantPart()
  {
    copConvexPolygons_.resize(static_cast<int>(stepPeriod_/samplingPeriod_));
    kinematicConvexPolygons_.resize(static_cast<int>(stepPeriod_/samplingPeriod_));
    //TODO: Complete
  }

  void HumanoidFeetSupervisor::init()
  {
    int N1 = leftFootModel_.getCopConvexPolygon().getNbVertices();
    int N2 = rightFootModel_.getCopConvexPolygon().getNbVertices();


    // Here we initialize the CoP convex polygon vector. The first element is the double
    // support convex polygon
    std::vector<Vector2> vec(N1 + N2);

    // We create a vector containing all vertices of both left and right CoP convex
    // polygon. All of them are centered on their belonging foot.

    // Inserting left cop polygon vertices
    for(int i=0; i<N1; ++i)
    {
      vec[i] = leftFootModel_.getCopConvexPolygon().getVertices()[i];
    }

    // Inserting right cop polygon vertices in local frame, i.e. the frame attached to
    // the left foot
    for(int i=0; i<N2; ++i)
    {
      //TODO: Rotations
      vec[i + N1] = rightFootModel_.getCopConvexPolygon().getVertices()[i]
          + Vector2(rightFootModel_.getStateX()(0), rightFootModel_.getStateY()(0))
          - Vector2(leftFootModel_.getStateX()(0), leftFootModel_.getStateY()(0));
    }

    copConvexPolygons_.push_back(ConvexPolygon(vec));

    // Foot convex polygon vector initialization. The right foot will be the first to fly.
    kinematicConvexPolygons_.push_back(leftFootModel_.getKinematicConvexPolygon());

    computeSelectionMatrix();
    computeFeetPosDynamic();
    computeRotationMatrix();
  }

  void HumanoidFeetSupervisor::computeSelectionMatrix()
  {
    selectionMatrices_.reset(getNbSamples(), getNbPreviewedSteps());
    //TODO: complete
  }

  void HumanoidFeetSupervisor::computeFeetPosDynamic()
  {
    feetPosDynamic_ = selectionMatrices_.toLinearDynamics();
  }

  void HumanoidFeetSupervisor::computeRotationMatrix()
  {
    rotationMatrix_.setZero(2*getNbSamples(), 2*getNbSamples());
    rotationMatrixT_.setZero(2*getNbSamples(), 2*getNbSamples());
    //TODO: complete. WARNING: rotation of -yaw
  }
}
