////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_walkgen.h
///\brief Main program for Humanoid
///\author de Gourcuff Martin
///\date 11/07/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef HUMANOID_WALKGEN_H
#define HUMANOID_WALKGEN_H

#include "type.h"
#include "model/lip_model.h"
#include "model/humanoid_foot_model.h"
#include "humanoid_feet_supervisor.h"
#include "function/humanoid_lip_com_velocity_tracking_objective.h"
#include "function/humanoid_lip_com_jerk_minimization_objective.h"
#include "function/humanoid_cop_centering_objective.h"
#include "function/humanoid_cop_constraint.h"
#include "function/humanoid_foot_constraint.h"
#include "solver/qpoases_solver.h"
#include <mpc-walkgen/humanoid_walkgen_abstract.h>

namespace MPCWalkgen
{

  class HumanoidWalkgen
  {
    public:
      HumanoidWalkgen();
      ~HumanoidWalkgen();

      void setMaximumNbOfSteps(int maximumNbOfSteps);

      void setNbSamples(int nbSamples);
      void setSamplingPeriod(Scalar samplingPeriod);
      void setStepPeriod(Scalar stepPeriod);

      void setLeftFootKinematicConvexPolygon(const ConvexPolygon& convexPolygon);
      void setRightFootKinematicConvexPolygon(const ConvexPolygon& convexPolygon);
      void setLeftFootCopConvexPolygon(const ConvexPolygon& convexPolygon);
      void setRightFootCopConvexPolygon(const ConvexPolygon& convexPolygon);

      void setVelRefInWorldFrame(const VectorX& velRef);

      void setLeftFootStateX(const VectorX& state);
      void setLeftFootStateY(const VectorX& state);
      void setLeftFootStateZ(const VectorX& state);
      void setRightFootStateX(const VectorX& state);
      void setRightFootStateY(const VectorX& state);
      void setRightFootStateZ(const VectorX& state);
      void setComStateX(const VectorX& state);
      void setComStateY(const VectorX& state);
      void setComStateZ(const VectorX& state);

      const VectorX& getLeftFootStateX() const;
      const VectorX& getLeftFootStateY() const;
      const VectorX& getLeftFootStateZ() const;
      const VectorX& getRightFootStateX() const;
      const VectorX& getRightFootStateY() const;
      const VectorX& getRightFootStateZ() const;
      const VectorX& getComStateX() const;
      const VectorX& getComStateY() const;
      const VectorX& getComStateZ() const;

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

      void setWeightings(const HumanoidWalkgenImpl::Weighting& weighting);
      void setConfig(const HumanoidWalkgenImpl::Config& config);

      bool solve(Scalar feedBackPeriod);

    private:
      /// \brief Convert CoP state in local frame into CoM jerk values in world frame
      void convertCopInLFtoJerkInWF();
      /// \brief Compute the constant parts of the QP matrices
      //Useless
      void computeConstantPart();

    private:
      LIPModel lipModel_;
      HumanoidFeetSupervisor feetSupervisor_;

      HumanoidLipComVelocityTrackingObjective velTrackingObj_;
      HumanoidLipComJerkMinimizationObjective jerkMinObj_;
      HumanoidCopCenteringObjective copCenteringObj_;

      HumanoidCopConstraint copConstraint_;
      HumanoidFootConstraint footConstraint_;

      /// \brief Vector containing all possible sizes of QPsolver
      std::vector<QPOasesSolver> qpoasesSolverVec_;

      VectorX dX_;
      /// \brief Solution of the QP problem: CoP position in local frame and
      ///        previewed footsteps positions in world frame.
      VectorX X_;
      /// \brief Transformed solution of the QP problem: CoM jerk in world frame and
      ///        previewed foosteps positions in world frame.
      VectorX transformedX_; //TODO: Change this ugly name


      /// \brief QPMatrices is a struct containing the matrices of the QP problem:
      ///        1/2*xT.H.x + xT.g
      ///        under the following constraints:
      ///        bl <= A.x <= bu
      ///        xl <= x <= xu
      ///        Here we use a vector containing all possible sizes of QPMatrices
      std::vector<QPMatrices> qpMatrixVec_;

      HumanoidWalkgenImpl::Weighting weighting_;
      HumanoidWalkgenImpl::Config config_;

      int maximumNbOfConstraints_;
      int maximumNbOfSteps_;
  };
}


#endif // MPC_WALKGEN_HUMANOID_WALKGEN_H
