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
  Phase::Phase()
    :phaseType_(DS)
    ,duration_(0.0)
  {}

  Phase::Phase(PhaseType phaseType,
               Scalar duration)
    :phaseType_(phaseType)
    ,duration_(duration)
  {}

  void SelectionMatrices::reset(
      int nbSamples,
      int nbPreviewedSteps)
  {
    assert(nbSamples>0);
    assert(nbPreviewedSteps>=0);

    V.setZero(nbSamples, nbPreviewedSteps);
    VT.setZero(nbPreviewedSteps, nbSamples);
    V0.setZero(nbSamples, 1);
  }

  LinearDynamic SelectionMatrices::toLinearDynamics()
  {
    LinearDynamic output;

    output.reset(V0.rows(), V0.cols(), V.cols());

    //TODO: check if this cast works
    output.U = V.cast<Scalar>();
    output.UT = VT.cast<Scalar>();
    output.S = V0.cast<Scalar>();

    return output;
  }


  // By default, the step period is set to 2 sampling periods, as the
  // transitional DS period already last one sampling period.
  HumanoidFeetSupervisor::HumanoidFeetSupervisor(int nbSamples,
                                                 Scalar samplingPeriod)
    :nbSamples_(nbSamples)
    ,samplingPeriod_(samplingPeriod)
    ,feedbackPeriod_(samplingPeriod_)
    ,leftFootModel_(nbSamples_, samplingPeriod_)
    ,rightFootModel_(nbSamples_, samplingPeriod_)
    ,copDSConvexPolygon_()
    ,nbPreviewedSteps_(0)
    ,stepPeriod_(2.0*samplingPeriod_)
    ,DSPeriod_(stepPeriod_)
    ,timeToNextPhase_(DSPeriod_)
    ,phaseTimer_(0.0)
    ,horizonTimer_(0.0)
    ,move_(false)
    ,phase_(Phase::DS, DSPeriod_)
  {
    init();
  }

  HumanoidFeetSupervisor::HumanoidFeetSupervisor()
    :nbSamples_(1)
    ,samplingPeriod_(1.0)
    ,feedbackPeriod_(samplingPeriod_)
    ,leftFootModel_(nbSamples_, samplingPeriod_)
    ,rightFootModel_(nbSamples_, samplingPeriod_)
    ,copDSConvexPolygon_()
    ,nbPreviewedSteps_(0)
    ,stepPeriod_(2.0*samplingPeriod_)
    ,DSPeriod_(stepPeriod_)
    ,timeToNextPhase_(DSPeriod_)
    ,phaseTimer_(0.0)
    ,horizonTimer_(0.0)
    ,move_(false)
    ,phase_(Phase::DS, DSPeriod_)
  {
    init();
  }

  HumanoidFeetSupervisor::~HumanoidFeetSupervisor()
  {}

  void HumanoidFeetSupervisor::setNbSamples(int nbSamples)
  {
    assert(nbSamples>0);

    nbSamples_ = nbSamples;
    leftFootModel_.setNbSamples(nbSamples);
    rightFootModel_.setNbSamples(nbSamples);

    computeConstantPart();
  }

  void HumanoidFeetSupervisor::setSamplingPeriod(Scalar samplingPeriod)
  {
    assert(samplingPeriod>=0);

    samplingPeriod_ = samplingPeriod;
    leftFootModel_.setSamplingPeriod(samplingPeriod);
    rightFootModel_.setSamplingPeriod(samplingPeriod);

    feedbackPeriod_ = samplingPeriod_;

    computeConstantPart();
  }

  void HumanoidFeetSupervisor::setStepPeriod(Scalar stepPeriod)
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

  void HumanoidFeetSupervisor::setInitialDoubleSupportLength(
      Scalar initialDoubleSupportLength)
  {
    //TODO: assert pour être sur que ce soit un multiple de samplingperiod?

    if(isInDS())
    {
      if(initialDoubleSupportLength < DSPeriod_ - EPSILON)
      {
        assert(timeToNextPhase_ + EPSILON > DSPeriod_ - initialDoubleSupportLength);
      }
      timeToNextPhase_ +=  initialDoubleSupportLength - DSPeriod_;
      timeline_.front().duration_ +=  initialDoubleSupportLength - DSPeriod_;
    }

    DSPeriod_ = initialDoubleSupportLength;

    computeConstantPart();
  }

  void HumanoidFeetSupervisor::setLeftFootKinematicConvexPolygon(
      const ConvexPolygon& convexPolygon)
  {
    leftFootModel_.setKinematicConvexPolygon(convexPolygon);

    computeConstantPart();
  }

  void HumanoidFeetSupervisor::setRightFootKinematicConvexPolygon(
      const ConvexPolygon &convexPolygon)
  {
    rightFootModel_.setKinematicConvexPolygon(convexPolygon);

    computeConstantPart();
  }

  void HumanoidFeetSupervisor::setLeftFootCopConvexPolygon(
      const ConvexPolygon& convexPolygon)
  {
    leftFootModel_.setCopConvexPolygon(convexPolygon);

    computeConstantPart();
  }

  void HumanoidFeetSupervisor::setRightFootCopConvexPolygon(
      const ConvexPolygon& convexPolygon)
  {
    rightFootModel_.setCopConvexPolygon(convexPolygon);

    computeConstantPart();
  }

  void HumanoidFeetSupervisor::setLeftFootStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    leftFootModel_.setStateX(state);

    computeConstantPart();
  }

  void HumanoidFeetSupervisor::setLeftFootStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    leftFootModel_.setStateY(state);

    computeConstantPart();
  }

  void HumanoidFeetSupervisor::setLeftFootStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    leftFootModel_.setStateZ(state);

    computeConstantPart();
  }

  void HumanoidFeetSupervisor::setRightFootStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    rightFootModel_.setStateX(state);

    computeConstantPart();
  }

  void HumanoidFeetSupervisor::setRightFootStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    rightFootModel_.setStateY(state);

    computeConstantPart();
  }

  void HumanoidFeetSupervisor::setRightFootStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==3);
    rightFootModel_.setStateZ(state);

    computeConstantPart();
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


  void HumanoidFeetSupervisor::setMove(bool move)
  {
    move_ = move;
  }

  int HumanoidFeetSupervisor::getMaximumNbOfCopConstraints()
  {
    // The maximum of constraints is reached when CoP convex polygons of both feet
    // are merged into one double support cop polygon.
    return std::max(leftFootModel_.getCopConvexPolygon().getNbVertices() +
                    rightFootModel_.getCopConvexPolygon().getNbVertices(),
                    0);
  }

  int HumanoidFeetSupervisor::getMaximumNbOfKinematicConstraints()
  {
    return std::max(leftFootModel_.getKinematicConvexPolygon().getNbVertices(),
                    rightFootModel_.getKinematicConvexPolygon().getNbVertices());
  }

  const ConvexPolygon& HumanoidFeetSupervisor::getCopConvexPolygon(int sampleIndex) const
  {
    switch (timeline_[phaseIndexFromSample_(sampleIndex)].phaseType_)
    {
    case Phase::DS:
      computeDSCopConvexPolygon();
      return copDSConvexPolygon_;

    case Phase::leftSS:
      return leftFootModel_.getCopConvexPolygon();

    case Phase::rightSS:
      return rightFootModel_.getCopConvexPolygon();
    }

    std::abort();
  }

  const ConvexPolygon& HumanoidFeetSupervisor::getKinematicConvexPolygon(int stepIndex) const
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
    case Phase::leftSS:
      return leftFootModel_.getKinematicConvexPolygon();

    case Phase::rightSS:
      return rightFootModel_.getKinematicConvexPolygon();

    case Phase::DS:; // avoid a compiler warning
    }

    std::abort();
  }

  //TODO: change for getSupportFootStateXAtPhase(int phaseIndex)?
  const VectorX& HumanoidFeetSupervisor::getSupportFootStateX() const
  {
    if(timeline_.front().phaseType_ == Phase::rightSS)
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


  const VectorX& HumanoidFeetSupervisor::getSupportFootStateY() const
  {
    if(timeline_.front().phaseType_ == Phase::rightSS)
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

  const int HumanoidFeetSupervisor::getNbOfCallsBeforeNextSample() const
  {
    Scalar timeToNextSample = timeToNextPhase_;

    while(timeToNextSample > samplingPeriod_ + EPSILON)
    {
      timeToNextSample -= samplingPeriod_;
    }

    return static_cast<int>((timeToNextSample + EPSILON)/feedbackPeriod_);
  }

  bool HumanoidFeetSupervisor::isInDS() const
  {
    if(timeline_.front().phaseType_ == Phase::DS)
    {
      return true;
    }

    return false;
  }

  void HumanoidFeetSupervisor::updateTimeline(VectorX& variable,
                                              Scalar feedBackPeriod)
  {
    assert(timeline_.size()>0);
    assert(variable.rows() >= 2*nbSamples_);
    feedbackPeriod_ = feedBackPeriod;

    // Check if the current phase needs to change
    if(timeToNextPhase_ < EPSILON)
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

    while(horizonTimer_ > -EPSILON)
    {
      if(phaseTimer_ < EPSILON)
      {
        processFSM(timeline_[phaseIndex]);

        // If the end of the timeline is reached, we add a  new phase,
        // otherwise the old one is overwritten
        if(phaseIndex == static_cast<int>(timeline_.size()) - 1)
        {
          timeline_.set_capacity(timeline_.capacity() + 1);
          timeline_.push_back(Phase(phase_));
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
        variable(2*nbSamples_ + 1) = variable(variable.rows()/2.0 + nbSamples_);
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

  void HumanoidFeetSupervisor::updateFeetStates(const VectorX& stepVec,
                                                Scalar feedBackPeriod)
  {
    stepVec_ = stepVec;
    Scalar flyingTime = timeline_.front().duration_ - samplingPeriod_;

    // This "if" statement checks if we are in a transitional double support
    // or in double support. If so, there is no update to do.
    if(!isInDS() &&
       (timeToNextPhase_ < flyingTime + EPSILON))
    {
      HumanoidFootModel* flyingFoot;

      switch (timeline_.front().phaseType_)
      {
      case Phase::leftSS:
        flyingFoot = &rightFootModel_;
        break;

      case Phase::rightSS:
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
      if(timeToNextPhase_ < flyingTime/2.0 + EPSILON)
      {
        flyingFoot->updateStateZ(Vector3::Zero(),
                                 timeToNextPhase_,
                                 feedBackPeriod);
      }
      else
      {
        flyingFoot->updateStateZ(Vector3(flyingFoot->getMaxHeight(), 0, 0),
                                 timeToNextPhase_ - flyingTime/2.0,
                                 feedBackPeriod);
      }
    }

    // Updating timeToNextPhase_
    timeToNextPhase_ -= feedBackPeriod;
  }

  void HumanoidFeetSupervisor::computeConstantPart()
  {
    //USELESS?
  }


  void HumanoidFeetSupervisor::init()
  {
    timeline_.set_capacity(1);
    setPhase(Phase::DS);
    timeline_.push_back(Phase(phase_));
  }

  void HumanoidFeetSupervisor::processFSM(const Phase& lastPhase)
  {
    // FSM of the walking generator
    if (lastPhase.phaseType_ == Phase::DS)
    {
      if(move_)
      {
        //DS -> SS
        setPhase(Phase::leftSS);
      }
      else
      {
        //DS -> DS
        setPhase(Phase::DS);
      }
    }
    else
    {
      ++nbPreviewedSteps_;

      if(!move_)
      {
        //SS -> DS
        setPhase(Phase::DS);
      }
      else
      {
        //SS -> SS
        if(lastPhase.phaseType_ == Phase::leftSS)
        {
          setPhase(Phase::rightSS);
        }
        else
        {
          setPhase(Phase::leftSS);
        }
      }
    }
  }

  void HumanoidFeetSupervisor::setPhase(Phase::PhaseType phaseType)
  {
    switch (phaseType)
    {
    case Phase::DS:
      phase_.phaseType_ = Phase::DS;
      phase_.duration_ = DSPeriod_;
      break;
    case Phase::leftSS:
      phase_.phaseType_ = Phase::leftSS;
      phase_.duration_ = stepPeriod_;
      break;
    case Phase::rightSS:
      phase_.phaseType_ = Phase::rightSS;
      phase_.duration_ = stepPeriod_;
      break;
    default:
      std::abort();
      break;
    }
  }

  void HumanoidFeetSupervisor::shortenStepVec(VectorX& variable) const
  {
    assert(variable.rows()%2 == 0);

    int newNbOfSteps = variable.rows()/2.0 - nbSamples_ - 1;
    VectorX save = variable;


    variable = save.segment(0, 2*nbSamples_ + 2*newNbOfSteps);

    variable.segment(2*nbSamples_, newNbOfSteps) =
        save.segment(2*nbSamples_ + 1,  newNbOfSteps);
    variable.segment(2*nbSamples_ + newNbOfSteps, newNbOfSteps) =
        save.segment(nbSamples_ + save.rows()/2 + 1,  newNbOfSteps);
  }


  void HumanoidFeetSupervisor::enlargeStepVec(VectorX& variable) const
  {
    assert(variable.rows()%2 == 0);

    int newNbOfSteps = variable.rows()/2.0 - nbSamples_ + 1;
    VectorX save = variable;

    // Here X_ is too short, so we add a new footstep position.
    // X coordinate: X coordinate of the middle of the leftmost and rightmost vertices of
    // the last footsteps CP
    // Y coordinate: Y coordinate of the middle of the lowest and highest vertices of
    // the last footsteps CP
    // Since the polygon is convex, this method ensures that the new foostep will respect
    // the constraints
    int newSize = 2*nbSamples_ + 2*newNbOfSteps;

    variable.conservativeResize(newSize);

    variable.segment(2*nbSamples_ + newNbOfSteps, newNbOfSteps - 1) =
        save.segment(save.rows()/2 + nbSamples_, newNbOfSteps - 1);

    const ConvexPolygon& kinCP = getKinematicConvexPolygon(newNbOfSteps - 1);

    std::vector<Vector2> p = kinCP.getVertices();

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
          getLeftFootStateX()(0) + (rightmostPointX + leftmostPointX)/2.0;

      variable(newSize - 1) =
          getLeftFootStateY()(0) + (lowestPointY + highestPointY)/2.0;
    }
    else
    {
      variable(2*nbSamples_ + newNbOfSteps - 1) =
          variable(2*nbSamples_ + newNbOfSteps - 2) + (rightmostPointX + leftmostPointX)/2.0;

      variable(newSize - 1) =
          variable(newSize - 2) + (lowestPointY + highestPointY)/2.0;
    }
  }

  void HumanoidFeetSupervisor::computeDSCopConvexPolygon() const
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
      if(timeline_[0].phaseType_ == Phase::leftSS)
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
      translationVec = (rightFootPos + leftFootPos)/2.0;
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

    copDSConvexPolygon_ = ConvexPolygon(copDSpoints_);
  }

  void HumanoidFeetSupervisor::computeSampleWeightMatrix()
  {
    Scalar timeToNextSample = timeToNextPhase_;

    while(timeToNextSample > samplingPeriod_ + EPSILON)
    {
      timeToNextSample -= samplingPeriod_;
    }

    sampleWeightMatrix_ = MatrixX::Identity(nbSamples_, nbSamples_);
    // The first sample weight balances the QP matrices element values as the remaining time
    // before the next QP sample varies if feedbackPeriod < samplingPeriod_.
    sampleWeightMatrix_(0, 0) = timeToNextSample/samplingPeriod_;
    // The last sample weight is only here for esthetic purposes
    sampleWeightMatrix_(nbSamples_-1, nbSamples_-1) =
        1.05 - sampleWeightMatrix_(0, 0);
  }

  void HumanoidFeetSupervisor::computeSelectionMatrix()
  {
    selectionMatrices_.reset(nbSamples_, nbPreviewedSteps_);
    //selectionMatrices_.V0.fill(1.0); //DEBUG

    phaseIndexFromSample_.setZero(nbSamples_);

    //Filling matrix V0
    int row = 0;
    phaseTimer_ = timeToNextPhase_;

    while ((phaseTimer_ > samplingPeriod_ + EPSILON) && (row < nbSamples_))
    {
      selectionMatrices_.V0(row) = 1;
      phaseTimer_ -= samplingPeriod_;
      ++row;
    }

    //Completing V0 and/or filling matrix V
    int col = 0;
    int phaseIndex = 1;
    horizonTimer_ = nbSamples_*samplingPeriod_ - timeToNextPhase_;

    while(horizonTimer_ > -EPSILON)
    {
      phaseTimer_ = timeline_[phaseIndex].duration_;

      while((phaseTimer_ > EPSILON) && (row < nbSamples_))
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

  void HumanoidFeetSupervisor::computeFeetPosDynamic()
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

  void HumanoidFeetSupervisor::computeRotationMatrix()
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
}
