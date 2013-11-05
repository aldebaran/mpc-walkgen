////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_feet_supervisor.h
///\brief Implement the supervisor that manage the FSM
///\author de Gourcuff Martin
///\date 22/08/13
///
////////////////////////////////////////////////////////////////////////////////

#ifndef HUMANOID_FEET_SUPERVISOR_H
#define HUMANOID_FEET_SUPERVISOR_H

#include "model/humanoid_foot_model.h"
#include "type.h"
#include <boost/circular_buffer.hpp>

namespace MPCWalkgen
{

  /// \brief This struct contains the type of phase the robot may be in:
  ///        -Double support (DS), when the robot has not started walking yet or is not
  ///         walking anymore
  ///        -Left simple support (LeftSS), when the support foot is the robot left foot
  ///        -Right simple support (RightSS), when the support foot is the robot right foot
  ///        This struct also contains the expected duration_ of the phase
  struct Phase
  {
      enum PhaseType
      {DS=0, leftSS, rightSS};

      Phase();

      Phase(PhaseType phaseType,
            Scalar duration);

      PhaseType phaseType_;
      Scalar duration_;
  };

  /// \brief Matrix V points out the correspondance between
  ///        the N samples and the M previewed steps:
  ///        V(i,j) = 1 if the ith sample match with the jth footstep,
  ///        V(i,j) = 0 otherwise.
  ///        Matrix V0 is the same but for the current step
  struct SelectionMatrices
  {
      void reset(int nbSamples,
                 int nbPreviewedSteps);
      LinearDynamic toLinearDynamics();

      Eigen::MatrixXi V;
      Eigen::MatrixXi VT;
      Eigen::MatrixXi V0;
  };

  class HumanoidFeetSupervisor
  {
    public:
      HumanoidFeetSupervisor(int nbSamples,
                             Scalar samplingPeriod);
      HumanoidFeetSupervisor();
      ~HumanoidFeetSupervisor();

      void setNbSamples(int nbSamples);
      void setSamplingPeriod(Scalar samplingPeriod);
      void setStepPeriod(Scalar stepPeriod);

      void setInitialDoubleSupportLength(Scalar initialDoubleSupportLength);

      void setLeftFootKinematicConvexPolygon(const ConvexPolygon& convexPolygon);
      void setRightFootKinematicConvexPolygon(const ConvexPolygon& convexPolygon);
      void setLeftFootCopConvexPolygon(const ConvexPolygon& convexPolygon);
      void setRightFootCopConvexPolygon(const ConvexPolygon& convexPolygon);

      void setLeftFootStateX(const VectorX& state);
      void setLeftFootStateY(const VectorX& state);
      void setLeftFootStateZ(const VectorX& state);
      void setRightFootStateX(const VectorX& state);
      void setRightFootStateY(const VectorX& state);
      void setRightFootStateZ(const VectorX& state);

      void setLeftFootMaxHeight(Scalar leftFootMaxHeight);
      void setRightFootMaxHeight(Scalar rightFootMaxHeight);

      void setLeftFootYawUpperBound(Scalar leftFootYawUpperBound);
      void setLeftFootYawLowerBound(Scalar leftFootYawLowerBound);
      void setRightFootYawUpperBound(Scalar rightFootYawUpperBound);
      void setRightFootYawLowerBound(Scalar rightFootYawLowerBound);

      void setLeftFootYawSpeedUpperBound(Scalar leftFootYawSpeedUpperBound);
      void setRightFootYawSpeedUpperBound(Scalar rightFootYawSpeedUpperBound);

      void setLeftFootYawAccelerationUpperBound(
          Scalar leftFootYawAccelerationUpperBound);
      void setRightFootYawAccelerationUpperBound(
          Scalar rightFootYawAccelerationUpperBound);

      void setMove(bool move);


      inline int getNbSamples() const
      {return nbSamples_;}

      inline Scalar getSamplingPeriod() const
      {return samplingPeriod_;}

      inline int getNbPreviewedSteps() const
      {return nbPreviewedSteps_;}

      inline int getStepPeriod() const
      {return stepPeriod_;}

      inline const MatrixX& getSampleWeightMatrix() const
      {return sampleWeightMatrix_;}

      inline const SelectionMatrices& getSelectionMatrices() const
      {return selectionMatrices_;}

      inline const LinearDynamic& getFeetPosLinearDynamic() const
      {return feetPosDynamic_;}

      inline const MatrixX& getRotationMatrix() const
      {return rotationMatrix_;}

      inline const MatrixX& getRotationMatrixT() const
      {return rotationMatrixT_;}

      inline const VectorX& getLeftFootStateX() const
      {return leftFootModel_.getStateX();}

      inline const VectorX& getLeftFootStateY() const
      {return leftFootModel_.getStateY();}

      inline const VectorX& getLeftFootStateZ() const
      {return leftFootModel_.getStateZ();}

      inline const VectorX& getRightFootStateX() const
      {return rightFootModel_.getStateX();}

      inline const VectorX& getRightFootStateY() const
      {return rightFootModel_.getStateY();}

      inline const VectorX& getRightFootStateZ() const
      {return rightFootModel_.getStateZ();}

      /// \brief Methods used to size the QP problem solvers and matrices vectors
      ///        The maximums values provided are for one sample and one step respectively,
      ///        not for the entire preview window
      int getMaximumNbOfCopConstraints();
      int getMaximumNbOfKinematicConstraints();

      /// \brief Return the CoP convex polygon at the (sampleIndex + 1)-th sample
      const ConvexPolygon& getCopConvexPolygon(int sampleIndex) const;
      /// \brief Return the kinematic convex polygon of the stepIndex-th previewed step
      const ConvexPolygon& getKinematicConvexPolygon(int stepIndex) const;
      /// \brief Return X state of the current support foot
      const VectorX& getSupportFootStateX() const;
      /// \brief Return Y state of the current support foot
      const VectorX& getSupportFootStateY() const;
      /// \brief Return the number of feedback period before the beginning next QP sample
      const int getNbOfCallsBeforeNextSample() const;

      /// \brief Return true if the current phase is a double support phase
      bool isInDS() const;


      /// \brief Update the footsteps timeline
      void updateTimeline(VectorX& variable,
                          Scalar feedBackPeriod);
      /// \brief Update left and right foot states
      void updateFeetStates(const VectorX& stepVec,
                            Scalar feedBackPeriod);
      void computeConstantPart();

    private:
      /// \brief Called by the constructor
      void init();
      /// \brief Return the next previewed phase that should follow lastPhase according
      ///        to the finite state machine.
      void processFSM(const Phase& lastPhase);
      /// \brief Set phase_ memebers accordingly to phaseType
      void setPhase(Phase::PhaseType phaseType);
      /// \brief Shorten the QP variable accordingly to the FSM
      void shortenStepVec(VectorX& variable) const;
      /// \brief Enlarge the QP variable accordingly to the FSM
      void enlargeStepVec(VectorX& variable) const;
      /// \brief For a given sample, this function gives the index of the corresponding
      ///        phase
      int phaseIndexFromSample(int sampleIndex) const;

      /// \brief Compute the double support convex polygon.
      ///        It is created by merging left foot CoP convex polygon and right foot CoP convex
      ///        polygon. It is centered on the robot support foot.
      ///        leftFootPos and rightFootPos must be given in world frame
      void computeDSCopConvexPolygon() const;

      void computeSampleWeightMatrix();
      void computeSelectionMatrix();
      void computeFeetPosDynamic();
      void computeRotationMatrix();

    private:
      int nbSamples_;
      Scalar samplingPeriod_;
      Scalar feedbackPeriod_;

      HumanoidFootModel leftFootModel_, rightFootModel_;
      mutable VectorX middleState_;
      mutable ConvexPolygon copDSConvexPolygon_;
      mutable std::vector<Vector2> copDSpoints_;

      int nbPreviewedSteps_;
      Scalar stepPeriod_;
      Scalar DSPeriod_;

      Scalar timeToNextPhase_;
      Scalar phaseTimer_;
      Scalar horizonTimer_;

      bool move_;

      VectorX stepVec_;

      SelectionMatrices selectionMatrices_;
      VectorX phaseIndexFromSample_;
      /// \brief Matrix of samples weights, used to provide more or less importance
      ///        to each sample in hessian and gradient computation
      MatrixX sampleWeightMatrix_;

      LinearDynamic feetPosDynamic_;

      MatrixX rotationMatrix_;
      MatrixX rotationMatrixT_;

      boost::circular_buffer<Phase> timeline_;
      Phase phase_;
  };

}
#endif // HUMANOID_FEET_SUPERVISOR_H
