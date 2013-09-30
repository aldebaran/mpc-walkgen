////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_walkgen_abstract.h
///\brief Main program for Humanoid
///\author de Gourcuff Martin
///\date 10/07/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_HUMANOID_WALKGEN_ABSTRACT_H
#define MPC_WALKGEN_HUMANOID_WALKGEN_ABSTRACT_H

#include <mpc-walkgen/shared_type.h>
#include <vector>

namespace MPCWalkgen
{

  class HumanoidWalkgen;

  class MPC_WALKGEN_NEW_API HumanoidWalkgenImpl
  {

    public:

      class Weighting
      {
        public:
          Weighting();

          Scalar velocityTracking;
          Scalar copCentering;
          Scalar jerkMinimization;
      };

      class Config
      {
        public:
          Config();

          bool withCopConstraints;
          bool withFeetConstraints;
      };

      HumanoidWalkgenImpl();
      ~HumanoidWalkgenImpl();

      /// \brief Set the number of QP samples within the preview window
      void setNbSamples(int nbSamples);
      /// \brief Set the QP sampling period in seconds
      void setSamplingPeriod(Scalar samplingPeriod);
      /// \brief Set the step period in seconds. It must be a multiple of
      ///  the QP sampling period
      void setStepPeriod(Scalar stepPeriod);

      ///  \brief Set the convex polygon of the positions that left and right foot can reach
      ///   according to their kinematic constraints (in local frame)
      void setLeftFootKinematicConvexPolygon(
          const std::vector<Vector2>& convexPolygon);
      void setRightFootKinematicConvexPolygon(
          const std::vector<Vector2>& convexPolygon);
      ///  \brief Set the simple support CoP convex polygon for left and right foot
      ///         in local frame
      void setLeftFootCopConvexPolygon(
          const std::vector<Vector2>& convexPolygon);
      void setRightFootCopConvexPolygon(
          const std::vector<Vector2>& convexPolygon);

      /// \brief Set the velocity reference in world frame
      void setVelRefInWorldFrame(const VectorX& velRef);


      /// \brief Getters and setters for both feet and CoM position, velocity
      /// and acceleration
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

      ///  \brief Set the maximum height that both feet can reach during a step
      void setLeftFootMaxHeight(Scalar leftFootMaxHeight);
      void setRightFootMaxHeight(Scalar rightFootMaxHeight);


      /// \brief Set upper and lower bounds for left and right foot yaw angles
      void setLeftFootYawUpperBound(Scalar leftFootYawUpperBound);
      void setLeftFootYawLowerBound(Scalar leftFootYawLowerBound);
      void setRightFootYawUpperBound(Scalar rightFootYawUpperBound);
      void setRightFootYawLowerBound(Scalar rightFootYawLowerBound);
      /// \brief Set upper bound for left and right foot angular speed around yaw axis
      void setLeftFootYawSpeedUpperBound(Scalar leftFootYawSpeedUpperBound);
      void setRightFootYawSpeedUpperBound(Scalar rightFootYawSpeedUpperBound);
      /// \brief Set upper bound for left and right foot angular acceleration around yaw axis
      void setLeftFootYawAccelerationUpperBound(Scalar leftFootYawAccelerationUpperBound);
      void setRightFootYawAccelerationUpperBound(Scalar rightFootYawAccelerationUpperBound);



      //TODO: doc
      void setWeightings(const Weighting& weighting);
      void setConfig(const Config& config);

      //TODO: more doc
      /// \brief solve the given QP problem of the form:
      ///        1/2*xT.H.x + xT.g
      ///        under the following constraints:
      ///        bl <= A.x <= bu
      ///        xl <= x <= xu
      /// \param feedBackPeriod: in seconds, the time between each call
      ///        of the MPC. This is also the period waited for before
      ///        new samples are sent to the actuators
      void solve(Scalar feedBackPeriod);

    private:
      HumanoidWalkgen* walkgen;
  };
}
#endif // MPC_WALKGEN_HUMANOID_WALKGEN_ABSTRACT_H
