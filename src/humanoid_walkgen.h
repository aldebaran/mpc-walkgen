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

      void setSamplingPeriod(Scalar samplingPeriod);
      void setNbSamples(int nbSamples);
      void setStepPeriod(Scalar stepPeriod);

      void setLeftFootKinematicHull(const std::vector<Vector3>& leftFootHull);
      void setRightFootKinematicHull(const std::vector<Vector3>& rightFootHull);
      void setSSCopHull(const std::vector<Vector3>& SSCopHull);
      void setDSCopHull(const std::vector<Vector3>& DSCopHull);

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

      void solve(Scalar feedBackPeriod);

    private:
      void computeConstantPart();

    private:
      HumanoidFootModel leftFootModel_, rightFootModel_;
      LIPModel lipModel_;

      HumanoidLipComVelocityTrackingObjective velTrackingObj_;
      HumanoidLipComJerkMinimizationObjective jerkMinObj_;
      HumanoidCopCenteringObjective copCenteringObj_;

      HumanoidCopConstraint copConstraint_;
      HumanoidFootConstraint footConstraint_;

      QPOasesSolver qpoasesSolver_;

      QPMatrices qpMatrix_;

      HumanoidWalkgenImpl::Weighting weighting_;
      HumanoidWalkgenImpl::Config config_;

  };
}


#endif // MPC_WALKGEN_HUMANOID_WALKGEN_H
