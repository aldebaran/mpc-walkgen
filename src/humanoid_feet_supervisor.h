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

  class HumanoidFeetSupervisor
  {
    public:
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
          Eigen::MatrixXi V0T;
      };

      HumanoidFeetSupervisor(const HumanoidFootModel& leftFoot,
                             const HumanoidFootModel& rightFoot,
                             int nbSamples,
                             Scalar samplingPeriod);
      HumanoidFeetSupervisor(const HumanoidFootModel& leftFoot,
                             const HumanoidFootModel& rightFoot);
      ~HumanoidFeetSupervisor();

      void setNbSamples(int nbSamples);
      void setSamplingPeriod(Scalar samplingPeriod);
      void setStepPeriod(Scalar stepPeriod);

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

      inline int getNbSamples() const
      {return nbSamples_;}

      inline Scalar getSamplingPeriod() const
      {return samplingPeriod_;}

      inline int getNbPreviewedSteps() const
      {return nbPreviewedSteps_;}

      inline int getStepPeriod() const
      {return stepPeriod_;}

      inline const SelectionMatrices& getSelectionMatrices() const
      {return selectionMatrices_;}

      inline const LinearDynamic& getFeetPosLinearDynamic() const
      {return feetPosDynamic_;}

      inline const MatrixX& getRotationMatrix() const
      {return rotationMatrix_;}

      inline const MatrixX& getRotationMatrixT() const
      {return rotationMatrixT_;}

      inline const boost::circular_buffer<ConvexPolygon>& getCopConvexPolygons() const
      {return copConvexPolygons_;}

      inline const boost::circular_buffer<ConvexPolygon>& getKinematicConvexPolygons() const
      {return kinematicConvexPolygons_;}

      /// \brief Methods used to size the QP problem solvers and matrices vectors
      int getMaximumNbOfSteps();
      int getMaximumNbOfCopConstraints();
      int getMaximumNbOfKinematicConstraints();

      void update();

      int sampleToStep(int sampleNb) const;
      void computeConstantPart();

    private:
      void init();
      void computeSelectionMatrix();
      void computeFeetPosDynamic();
      void computeRotationMatrix();

      HumanoidFootModel leftFootModel_, rightFootModel_;

      int nbSamples_;
      Scalar samplingPeriod_;

      int nbPreviewedSteps_;
      Scalar stepPeriod_;

      SelectionMatrices selectionMatrices_;
      LinearDynamic feetPosDynamic_;
      MatrixX rotationMatrix_;
      MatrixX rotationMatrixT_;



      /// \brief Circular buffer of the CoP convex polygons for each step in local frame.
      ///        It contains nbPreviewedSteps_ + 1 elements and its maximum size is
      boost::circular_buffer<ConvexPolygon> copConvexPolygons_;
      /// \brief Circular buffer of the Kinematic convex polygons for each step in world frame.
      ///        It is vector of size nbPreviewedSteps_ + 1
      boost::circular_buffer<ConvexPolygon> kinematicConvexPolygons_;
  };

}
#endif // HUMANOID_FEET_SUPERVISOR_H
