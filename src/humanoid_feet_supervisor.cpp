////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_feet_supervisor.h
///\brief Implement the supervisor that manage the FSM
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////
#include <mpc-walkgen/humanoid_feet_supervisor.h>
#include <mpc-walkgen/constant.h>
#include "macro.h"

namespace MPCWalkgen
{
  template <typename Scalar>
  Phase<Scalar>::Phase()
    :phaseType_(DS)
    ,duration_(0.0)
  {}

  template <typename Scalar>
  Phase<Scalar>::Phase(PhaseType phaseType,
               Scalar duration)
    :phaseType_(phaseType)
    ,duration_(duration)
  {}

  template <typename Scalar>
  void SelectionMatrices<Scalar>::reset(
      int nbSamples,
      int nbPreviewedSteps)
  {
    assert(nbSamples>0);
    assert(nbPreviewedSteps>=0);

    V.setZero(nbSamples, nbPreviewedSteps);
    VT.setZero(nbPreviewedSteps, nbSamples);
    V0.setZero(nbSamples, 1);
  }

  template <typename Scalar>
  LinearDynamic<Scalar> SelectionMatrices<Scalar>::toLinearDynamics()
  {
    LinearDynamic<Scalar> output;

    output.reset(V0.rows(), V0.cols(), V.cols());

    //TODO: check if this cast works
    output.U = V.cast<Scalar>();
    output.UT = VT.cast<Scalar>();
    output.S = V0.cast<Scalar>();

    return output;
  }


  // By default, the step period is set to 2 sampling periods, as the
  // transitional DS period already last one sampling period.
  template <typename Scalar>
  HumanoidFeetSupervisor<Scalar>::HumanoidFeetSupervisor(int nbSamples,
                                                 Scalar samplingPeriod)
    :nbSamples_(nbSamples)
    ,samplingPeriod_(samplingPeriod)
    ,feedbackPeriod_(samplingPeriod_)
    ,leftFootModel_(nbSamples_, samplingPeriod_)
    ,rightFootModel_(nbSamples_, samplingPeriod_)
    ,copDSConvexPolygon_()
    ,nbPreviewedSteps_(0)
    ,stepPeriod_(2*samplingPeriod_)
    ,DSPeriod_(stepPeriod_)
    ,timeToNextPhase_(DSPeriod_)
    ,phaseTimer_(0)
    ,horizonTimer_(0)
    ,move_(false)
    ,phase_(Phase<Scalar>::DS, DSPeriod_)
  {
    init();
  }

  template <typename Scalar>
  HumanoidFeetSupervisor<Scalar>::HumanoidFeetSupervisor()
    :nbSamples_(1)
    ,samplingPeriod_(1)
    ,feedbackPeriod_(samplingPeriod_)
    ,leftFootModel_(nbSamples_, samplingPeriod_)
    ,rightFootModel_(nbSamples_, samplingPeriod_)
    ,copDSConvexPolygon_()
    ,nbPreviewedSteps_(0)
    ,stepPeriod_(2*samplingPeriod_)
    ,DSPeriod_(stepPeriod_)
    ,timeToNextPhase_(DSPeriod_)
    ,phaseTimer_(0)
    ,horizonTimer_(0)
    ,move_(false)
    ,phase_(Phase<Scalar>::DS, DSPeriod_)
  {
    init();
  }

  template <typename Scalar>
  HumanoidFeetSupervisor<Scalar>::~HumanoidFeetSupervisor()
  {}

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setNbSamples(int nbSamples)
  {
    assert(nbSamples>0);

    nbSamples_ = nbSamples;
    leftFootModel_.setNbSamples(nbSamples);
    rightFootModel_.setNbSamples(nbSamples);

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setSamplingPeriod(Scalar samplingPeriod)
  {
    assert(samplingPeriod>=0);

    samplingPeriod_ = samplingPeriod;
    leftFootModel_.setSamplingPeriod(samplingPeriod);
    rightFootModel_.setSamplingPeriod(samplingPeriod);

    feedbackPeriod_ = samplingPeriod_;

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setStepPeriod(Scalar stepPeriod)
  {
    assert(stepPeriod>0);

    if(!isInDS())
    {
      assert(timeToNextPhase_ + stepPeriod - stepPeriod_>0);

      timeToNextPhase_ +=  stepPeriod - stepPeriod_;
      timeline_.front().duration_  +=  stepPeriod - stepPeriod_;
    }

    stepPeriod_ = stepPeriod;

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setInitialDoubleSupportLength(
      Scalar initialDoubleSupportLength)
  {
    //TODO: assert pour être sur que ce soit un multiple de samplingperiod?

    if(isInDS())
    {
      if(initialDoubleSupportLength < DSPeriod_ - Constant<Scalar>::EPSILON)
      {
        assert(timeToNextPhase_+Constant<Scalar>::EPSILON > DSPeriod_ - initialDoubleSupportLength);
      }
      timeToNextPhase_ +=  initialDoubleSupportLength - DSPeriod_;
      timeline_.front().duration_ +=  initialDoubleSupportLength - DSPeriod_;
    }

    DSPeriod_ = initialDoubleSupportLength;

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setLeftFootKinematicConvexPolygon(
      const ConvexPolygon<Scalar>& convexPolygon)
  {
    leftFootModel_.setKinematicConvexPolygon(convexPolygon);

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setRightFootKinematicConvexPolygon(
      const ConvexPolygon<Scalar> &convexPolygon)
  {
    rightFootModel_.setKinematicConvexPolygon(convexPolygon);

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setLeftFootCopConvexPolygon(
      const ConvexPolygon<Scalar>& convexPolygon)
  {
    leftFootModel_.setCopConvexPolygon(convexPolygon);

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setRightFootCopConvexPolygon(
      const ConvexPolygon<Scalar>& convexPolygon)
  {
    rightFootModel_.setCopConvexPolygon(convexPolygon);

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setLeftFootStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    leftFootModel_.setStateX(state);

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setLeftFootStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    leftFootModel_.setStateY(state);

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setLeftFootStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    leftFootModel_.setStateZ(state);

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setRightFootStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    rightFootModel_.setStateX(state);

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setRightFootStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    rightFootModel_.setStateY(state);

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setRightFootStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    rightFootModel_.setStateZ(state);

    computeConstantPart();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setLeftFootMaxHeight(
      Scalar leftFootMaxHeight)
  {
    leftFootModel_.setMaxHeight(
          leftFootMaxHeight);
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setRightFootMaxHeight(
      Scalar rightFootMaxHeight)
  {
    rightFootModel_.setMaxHeight(
          rightFootMaxHeight);
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setLeftFootYawUpperBound(
      Scalar leftFootYawUpperBound)
  {
    leftFootModel_.setHipYawUpperBound(
          leftFootYawUpperBound);
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setLeftFootYawLowerBound(
      Scalar leftFootYawLowerBound)
  {
    leftFootModel_.setHipYawLowerBound(
          leftFootYawLowerBound);
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setRightFootYawUpperBound(
      Scalar rightFootYawUpperBound)
  {
    rightFootModel_.setHipYawUpperBound(
          rightFootYawUpperBound);
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setRightFootYawLowerBound(
      Scalar rightFootYawLowerBound)
  {
    rightFootModel_.setHipYawLowerBound(
          rightFootYawLowerBound);
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setLeftFootYawSpeedUpperBound(
      Scalar leftFootYawSpeedUpperBound)
  {
    leftFootModel_.setHipYawSpeedUpperBound(
          leftFootYawSpeedUpperBound);
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setRightFootYawSpeedUpperBound(
      Scalar rightFootYawSpeedUpperBound)
  {
    rightFootModel_.setHipYawSpeedUpperBound(
          rightFootYawSpeedUpperBound);
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setLeftFootYawAccelerationUpperBound(
      Scalar leftFootYawAccelerationUpperBound)
  {
    leftFootModel_.setHipYawAccelerationUpperBound(
          leftFootYawAccelerationUpperBound);
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setRightFootYawAccelerationUpperBound(
      Scalar rightFootYawAccelerationUpperBound)
  {
    rightFootModel_.setHipYawAccelerationUpperBound(
          rightFootYawAccelerationUpperBound);
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setMove(bool move)
  {
    move_ = move;
  }

  template <typename Scalar>
  int HumanoidFeetSupervisor<Scalar>::getMaximumNbOfCopConstraints()
  {
    // The maximum of constraints is reached when CoP convex polygons of both feet
    // are merged into one double support cop polygon.
    return std::max(leftFootModel_.getCopConvexPolygon().getNbVertices() +
                    rightFootModel_.getCopConvexPolygon().getNbVertices(),
                    0);
  }

  template <typename Scalar>
  int HumanoidFeetSupervisor<Scalar>::getMaximumNbOfKinematicConstraints()
  {
    return std::max(leftFootModel_.getKinematicConvexPolygon().getNbVertices(),
                    rightFootModel_.getKinematicConvexPolygon().getNbVertices());
  }

  template <typename Scalar>
  const ConvexPolygon<Scalar>&
  HumanoidFeetSupervisor<Scalar>::getCopConvexPolygon(int sampleIndex) const
  {
    switch (timeline_[phaseIndexFromSample_(sampleIndex)].phaseType_)
    {
    case Phase<Scalar>::DS:
      computeDSCopConvexPolygon();
      return copDSConvexPolygon_;

    case Phase<Scalar>::leftSS:
      return leftFootModel_.getCopConvexPolygon();

    case Phase<Scalar>::rightSS:
      return rightFootModel_.getCopConvexPolygon();
    }

    std::abort();
    // avoid a compiler warning
    return leftFootModel_.getCopConvexPolygon();
  }

  template <typename Scalar>
  const ConvexPolygon<Scalar>&
  HumanoidFeetSupervisor<Scalar>::getKinematicConvexPolygon(int stepIndex) const
  {
    assert(stepIndex<nbPreviewedSteps_);
    // In double support, the first previewed step is actually the third element of the timeline,
    // It is then constrained by the timeline second element's kinetic convex polygon
    if(isInDS() && move_)
    {
      ++stepIndex;
    }

    switch (timeline_[stepIndex].phaseType_)
    {
    case Phase<Scalar>::leftSS:
      return leftFootModel_.getKinematicConvexPolygon();

    case Phase<Scalar>::rightSS:
      return rightFootModel_.getKinematicConvexPolygon();

    case Phase<Scalar>::DS:; // avoid a compiler warning
    }
    std::abort();
    // avoid a compiler warning
    return leftFootModel_.getKinematicConvexPolygon();
  }

  //TODO: change for getSupportFootStateXAtPhase(int phaseIndex)?
  template <typename Scalar>
  const typename Type<Scalar>::VectorX& HumanoidFeetSupervisor<Scalar>::getSupportFootStateX() const
  {
    if(timeline_.front().phaseType_ == Phase<Scalar>::rightSS)
    {
      return rightFootModel_.getStateX();
    }
    else if(isInDS() && !move_)
    {
      middleState_ = (rightFootModel_.getStateX() + leftFootModel_.getStateX())/2.0;
      return middleState_;
    }
    return leftFootModel_.getStateX();
  }

  template <typename Scalar>
  const typename Type<Scalar>::VectorX& HumanoidFeetSupervisor<Scalar>::getSupportFootStateY() const
  {
    if(timeline_.front().phaseType_ == Phase<Scalar>::rightSS)
    {
      return rightFootModel_.getStateY();
    }
    else if(isInDS() && !move_)
    {
      middleState_ = (rightFootModel_.getStateY() + leftFootModel_.getStateY())/2.0;
      return middleState_;
    }
    return leftFootModel_.getStateY();
  }

  template <typename Scalar>
  int HumanoidFeetSupervisor<Scalar>::getNbOfCallsBeforeNextSample() const
  {
    Scalar timeToNextSample = timeToNextPhase_;

    while(timeToNextSample > samplingPeriod_ + Constant<Scalar>::EPSILON)
    {
      timeToNextSample -= samplingPeriod_;
    }

    return static_cast<int>((timeToNextSample + Constant<Scalar>::EPSILON)/feedbackPeriod_);
  }

  template <typename Scalar>
  bool HumanoidFeetSupervisor<Scalar>::isInDS() const
  {
    if(timeline_.front().phaseType_ == Phase<Scalar>::DS)
    {
      return true;
    }

    return false;
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::updateTimeline(VectorX& variable,
                                              Scalar feedBackPeriod)
  {
    assert(timeline_.size()>0);
    assert(variable.rows() >= 2*nbSamples_);
    feedbackPeriod_ = feedBackPeriod;

    // Check if the current phase needs to change
    if(timeToNextPhase_ < Constant<Scalar>::EPSILON)
    {
      if(!isInDS())
      {
        shortenStepVec(variable);
      }

      timeline_.pop_front();
      timeline_.set_capacity(timeline_.capacity() - 1);

      timeToNextPhase_ = timeline_.front().duration_;
    }

    // Generate the new timeline from the current phase
    int phaseIndex = 0;
    nbPreviewedSteps_ = 0;
    // Reseting timers
    phaseTimer_ = timeToNextPhase_;
    horizonTimer_ = nbSamples_*samplingPeriod_;

    while(horizonTimer_ > -Constant<Scalar>::EPSILON)
    {
      if(phaseTimer_ < Constant<Scalar>::EPSILON)
      {
        processFSM(timeline_[phaseIndex]);

        // If the end of the timeline is reached, we add a  new phase,
        // otherwise the old one is overwritten
        if(phaseIndex == static_cast<int>(timeline_.size()) - 1)
        {
          timeline_.set_capacity(timeline_.capacity() + 1);
          timeline_.push_back(Phase<Scalar>(phase_));
        }
        else
        {
          timeline_[phaseIndex + 1].phaseType_ = phase_.phaseType_;
          timeline_[phaseIndex + 1].duration_ = phase_.duration_;
        }

        phaseTimer_ = timeline_[phaseIndex + 1].duration_;
        ++phaseIndex;
      }

      // This assure that variable size is big enough when
      while(variable.rows() < 2*nbSamples_ + 2*nbPreviewedSteps_)
      {
        enlargeStepVec(variable);
      }
      // When the user set move_ to false this checks if the QP variable size must be reduce.
      if(!move_ && (variable.rows() > 2*nbSamples_ + 2))
      {
        variable(2*nbSamples_ + 1) = variable(variable.rows()/2 + nbSamples_);
        variable.conservativeResize(2*nbSamples_ + 2);
      }

      // Decrementing timers
      phaseTimer_ -= samplingPeriod_;
      horizonTimer_ -= samplingPeriod_;
    }

    //Removing useless phases
    timeline_.set_capacity(phaseIndex + 1);

    stepVec_ = variable.segment(2*nbSamples_, variable.rows() - 2*nbSamples_);

    computeSampleWeightMatrix();
    computeSelectionMatrix();
    computeFeetPosDynamic();
    computeRotationMatrix();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::updateFeetStates(const VectorX& stepVec,
                                                Scalar feedBackPeriod)
  {
    stepVec_ = stepVec;
    Scalar flyingTime = timeline_.front().duration_ - samplingPeriod_;

    // This "if" statement checks if we are in a transitional double support
    // or in double support. If so, there is no update to do.
    if(!isInDS() &&
       (timeToNextPhase_ < flyingTime + Constant<Scalar>::EPSILON))
    {
      HumanoidFootModel<Scalar>* flyingFoot;

      switch (timeline_.front().phaseType_)
      {
      case Phase<Scalar>::leftSS:
        flyingFoot = &rightFootModel_;
        break;

      case Phase<Scalar>::rightSS:
        flyingFoot = &leftFootModel_;
        break;

      default:
        std::abort();
        break;
      }

      // Updating X and Y states for the flying foot
      flyingFoot->updateStateX(Vector3(stepVec(0), 0, 0),
                               timeToNextPhase_,
                               feedBackPeriod);
      flyingFoot->updateStateY(Vector3(stepVec(nbPreviewedSteps_), 0, 0),
                               timeToNextPhase_,
                               feedBackPeriod);

      //Updating Z state for the flying foot, depending whether the foot is going up or down
      if(timeToNextPhase_ < flyingTime*0.5f + Constant<Scalar>::EPSILON)
      {
        flyingFoot->updateStateZ(Vector3::Zero(),
                                 timeToNextPhase_,
                                 feedBackPeriod);
      }
      else
      {
        flyingFoot->updateStateZ(Vector3(flyingFoot->getMaxHeight(), 0, 0),
                                 timeToNextPhase_ - flyingTime*0.5f,
                                 feedBackPeriod);
      }
    }

    // Updating timeToNextPhase_
    timeToNextPhase_ -= feedBackPeriod;
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::computeConstantPart()
  {
    //USELESS?
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::init()
  {
    timeline_.set_capacity(1);
    setPhase(Phase<Scalar>::DS);
    timeline_.push_back(Phase<Scalar>(phase_));
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::processFSM(const Phase<Scalar>& lastPhase)
  {
    // FSM of the walking generator
    if (lastPhase.phaseType_ == Phase<Scalar>::DS)
    {
      if(move_)
      {
        //DS -> SS
        setPhase(Phase<Scalar>::leftSS);
      }
      else
      {
        //DS -> DS
        setPhase(Phase<Scalar>::DS);
      }
    }
    else
    {
      ++nbPreviewedSteps_;

      if(!move_)
      {
        //SS -> DS
        setPhase(Phase<Scalar>::DS);
      }
      else
      {
        //SS -> SS
        if(lastPhase.phaseType_ == Phase<Scalar>::leftSS)
        {
          setPhase(Phase<Scalar>::rightSS);
        }
        else
        {
          setPhase(Phase<Scalar>::leftSS);
        }
      }
    }
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::setPhase(typename Phase<Scalar>::PhaseType phaseType)
  {
    switch (phaseType)
    {
    case Phase<Scalar>::DS:
      phase_.phaseType_ = Phase<Scalar>::DS;
      phase_.duration_ = DSPeriod_;
      break;
    case Phase<Scalar>::leftSS:
      phase_.phaseType_ = Phase<Scalar>::leftSS;
      phase_.duration_ = stepPeriod_;
      break;
    case Phase<Scalar>::rightSS:
      phase_.phaseType_ = Phase<Scalar>::rightSS;
      phase_.duration_ = stepPeriod_;
      break;
    default:
      std::abort();
      break;
    }
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::shortenStepVec(VectorX& variable) const
  {
    assert(variable.rows()%2 == 0);

    int newNbOfSteps = variable.rows()/2 - nbSamples_ - 1;
    VectorX save = variable;


    variable = save.segment(0, 2*nbSamples_ + 2*newNbOfSteps);

    variable.segment(2*nbSamples_, newNbOfSteps) =
        save.segment(2*nbSamples_ + 1,  newNbOfSteps);
    variable.segment(2*nbSamples_ + newNbOfSteps, newNbOfSteps) =
        save.segment(nbSamples_ + save.rows()/2 + 1,  newNbOfSteps);
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::enlargeStepVec(VectorX& variable) const
  {
    assert(variable.rows()%2 == 0);

    const int newNbOfSteps = variable.rows()/2 - nbSamples_ + 1;
    VectorX save = variable;

    // Here X_ is too short, so we add a new footstep position.
    // X coordinate: X coordinate of the middle of the leftmost and rightmost vertices of
    // the last footsteps CP
    // Y coordinate: Y coordinate of the middle of the lowest and highest vertices of
    // the last footsteps CP
    // Since the polygon is convex, this method ensures that the new foostep will respect
    // the constraints
    const int newSize = 2*nbSamples_ + 2*newNbOfSteps;

    variable.conservativeResize(newSize);

    variable.segment(2*nbSamples_ + newNbOfSteps, newNbOfSteps - 1) =
        save.segment(save.rows()/2 + nbSamples_, newNbOfSteps - 1);

    const ConvexPolygon<Scalar>& kinCP = getKinematicConvexPolygon(newNbOfSteps - 1);

    const vectorOfVector2& p = kinCP.getVertices();

    assert(p.size()>0);

    Scalar leftmostPointX = p[0](0);
    Scalar rightmostPointX = p[0](0);

    Scalar lowestPointY = p[0](1);
    Scalar highestPointY = p[0](1);

    for(size_t j=1; j<p.size(); ++j)
    {
      if(leftmostPointX > p[j](0))
      {
        leftmostPointX = p[j](0);
      }
      if(rightmostPointX < p[j](0))
      {
        rightmostPointX = p[j](0);
      }

      if(lowestPointY > p[j](1))
      {
        lowestPointY = p[j](1);
      }
      if(highestPointY < p[j](1))
      {
        highestPointY = p[j](1);
      }
    }

    // Adding x and y coordinates of the new footstep position
    if(isInDS() && (nbPreviewedSteps_==1))
    {
      variable(2*nbSamples_ + newNbOfSteps - 1) =
          getLeftFootStateX()(0) + (rightmostPointX + leftmostPointX)/2;

      variable(newSize - 1) =
          getLeftFootStateY()(0) + (lowestPointY + highestPointY)/2;
    }
    else
    {
      variable(2*nbSamples_ + newNbOfSteps - 1) =
          variable(2*nbSamples_ + newNbOfSteps - 2) + (rightmostPointX + leftmostPointX)/2;

      variable(newSize - 1) =
          variable(newSize - 2) + (lowestPointY + highestPointY)/2;
    }
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::computeDSCopConvexPolygon() const
  {
    int N1 = leftFootModel_.getCopConvexPolygon().getNbVertices();
    int N2 = rightFootModel_.getCopConvexPolygon().getNbVertices();

    copDSpoints_.resize(N1 + N2);

    Vector2 leftFootPos(leftFootModel_.getStateX()(0),
                        leftFootModel_.getStateY()(0));
    Vector2 rightFootPos(rightFootModel_.getStateX()(0),
                         rightFootModel_.getStateY()(0));
    Vector2 translationVec(leftFootPos);

    if(!isInDS() && !move_)
    {
      if(timeline_[0].phaseType_ == Phase<Scalar>::leftSS)
      {
        rightFootPos(0) = stepVec_(0);
        rightFootPos(1) = stepVec_(nbPreviewedSteps_);
        translationVec = rightFootPos;
      }
      else
      {
        leftFootPos(0) = stepVec_(0);
        leftFootPos(1) = stepVec_(nbPreviewedSteps_);
        translationVec = leftFootPos;
      }
    }
    else if(isInDS() && !move_)
    {
      translationVec = (rightFootPos + leftFootPos)/2;
    }


    // Inserting left cop polygon vertices
    for(int i=0; i<N1; ++i)
    {
      copDSpoints_[i] = leftFootModel_.getCopConvexPolygon().getVertices()[i]
          + leftFootPos
          - translationVec;
    }

    // Inserting right cop polygon vertices expressed in left foot frame
    for(int i=0; i<N2; ++i)
    {
      //TODO: Rotations
      copDSpoints_[i + N1] = rightFootModel_.getCopConvexPolygon().getVertices()[i]
          + rightFootPos
          - translationVec;
    }

    copDSConvexPolygon_ = ConvexPolygon<Scalar>(copDSpoints_);
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::computeSampleWeightMatrix()
  {
    Scalar timeToNextSample = timeToNextPhase_;

    while(timeToNextSample > samplingPeriod_ + Constant<Scalar>::EPSILON)
    {
      timeToNextSample -= samplingPeriod_;
    }

    sampleWeightMatrix_ = MatrixX::Identity(nbSamples_, nbSamples_);
    // The first sample weight balances the QP matrices element values as the remaining time
    // before the next QP sample varies if feedbackPeriod < samplingPeriod_.
    sampleWeightMatrix_(0, 0) = timeToNextSample/samplingPeriod_;
    // The last sample weight is only here for esthetic purposes
    sampleWeightMatrix_(nbSamples_-1, nbSamples_-1) =
        1.05f - sampleWeightMatrix_(0, 0);
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::computeSelectionMatrix()
  {
    selectionMatrices_.reset(nbSamples_, nbPreviewedSteps_);

    phaseIndexFromSample_.setZero(nbSamples_);

    //Filling matrix V0
    int row = 0;
    phaseTimer_ = timeToNextPhase_;

    while ((phaseTimer_ > samplingPeriod_ + Constant<Scalar>::EPSILON) && (row < nbSamples_))
    {
      selectionMatrices_.V0(row) = 1;
      phaseTimer_ -= samplingPeriod_;
      ++row;
    }

    //Completing V0 and/or filling matrix V
    int col = 0;
    int phaseIndex = 1;
    horizonTimer_ = nbSamples_*samplingPeriod_ - timeToNextPhase_;

    while(horizonTimer_ > -Constant<Scalar>::EPSILON)
    {
      phaseTimer_ = timeline_[phaseIndex].duration_;

      while((phaseTimer_ > Constant<Scalar>::EPSILON) && (row < nbSamples_))
      {
        // Check for the special case where the timeline second element does not
        // count for a step because we are in double support
        if(isInDS() && (phaseIndex==1 || !move_))
        {
          selectionMatrices_.V0(row) = 1;
        }
        else
        {
          selectionMatrices_.V(row, col) = 1;
        }

        phaseIndexFromSample_(row) = phaseIndex;

        ++row;
        phaseTimer_ -= samplingPeriod_;
        horizonTimer_ -= samplingPeriod_;
      }

      if(move_ && (!isInDS() || phaseIndex>1))
      {
        ++col;
      }

      ++phaseIndex;
    }

    selectionMatrices_.VT = selectionMatrices_.V.transpose();
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::computeFeetPosDynamic()
  {
    feetPosDynamic_ = selectionMatrices_.toLinearDynamics();

    for(int i=0; i<selectionMatrices_.V0.rows(); ++i)
    {
      assert(feetPosDynamic_.S(i)==selectionMatrices_.V0(i));
      for(int j=0; j<selectionMatrices_.V.cols(); j++)
      {
        assert(feetPosDynamic_.U(i, j)==selectionMatrices_.V(i, j));
        assert(feetPosDynamic_.UT(j, i)==selectionMatrices_.VT(j, i));
      }
    }
  }

  template <typename Scalar>
  void HumanoidFeetSupervisor<Scalar>::computeRotationMatrix()
  {
    rotationMatrix_.setZero(2*nbSamples_, 2*nbSamples_);

    //TODO: complete. WARNING: rotation of -yaw
    for(int i=0; i<nbSamples_; ++i)
    {
      rotationMatrix_(i, i) = 1.0;
      rotationMatrix_(i + nbSamples_, i + nbSamples_) = 1.0;
    }

    rotationMatrixT_ = rotationMatrix_.transpose();
  }

  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(HumanoidFeetSupervisor);
}
