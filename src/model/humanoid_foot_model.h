////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_foot_model.h
///\brief Implement of a foot model
///\author de Gourcuff Martin
///\date 11/07/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_HUMANOID_FOOT_MODEL_H
#define MPC_WALKGEN_HUMANOID_FOOT_MODEL_H

#include "../type.h"

namespace MPCWalkgen{
  class HumanoidFootModel
  {
    public:
      /// \brief Matrix V points out the correspondance between
      ///        the N samples and the M previewed steps:
      ///        V(i,j) = 1 if the ith sample match with the jth footstep,
      ///        V(i,j) = 0 otherwise.
      ///        Matrix V0 is the same but for the current step
      struct SelectionMatrix
      {
          void reset(int nbSamples,
                     int nbPreviewedSteps);
          LinearDynamic toLinearDynamics();

          Eigen::MatrixXi V;
          Eigen::MatrixXi VT;
          Eigen::MatrixXi V0;
          Eigen::MatrixXi V0T;
      };
      /// \brief Structure to store every kinematic limit the foot is
      ///        constraint to.
      struct  KinematicLimits
      {
          KinematicLimits();

          Scalar hipYawUpperBound_;
          Scalar hipYawSpeedUpperBound_;
          Scalar hipYawAccelerationUpperBound_;
          Scalar hipYawLowerBound_;
          Scalar maxHeight_;

          Hull kinematicHull_;
      };

      HumanoidFootModel(int nbSamples,
                        Scalar samplingPeriod,
                        int nbPreviewedSteps);
      HumanoidFootModel();


      ~HumanoidFootModel();

      /// \brief Set the number of samples for this dynamic
      void setNbSamples(int nbSamples);

      /// \brief Get the number of samples for this dynamic
      inline int getNbSamples() const
      {return nbSamples_;}

      /// \brief Set the sampling period
      void setSamplingPeriod(Scalar samplingPeriod);

      /// \brief Get the sampling period
      inline Scalar getSamplingPeriod() const
      {return samplingPeriod_;}

      /// \brief Get the number of previewed steps for which this foor
      ///        will be the support foot
      inline int getNbPreviewedSteps() const
      {return nbPreviewedSteps_;}

      /// \brief Get the double support hull in which the CoP
      ///        must remain
      inline const Hull& getCopDSHull() const
      {return copDSHull_;}

      /// \brief Get the double support hull
      inline void setCopDSHull(const Hull& CopDSHull)
      {
        assert(CopDSHull.p.size()==4);
        copDSHull_=CopDSHull;
      }

      /// \brief Get the simple support hull in which the CoP
      ///        must remain
      inline const Hull& getCopSSHull() const
      {return copSSHull_;}

      /// \brief Get the simple support hull
      inline void setCopSSHull(const Hull& CopSSHull)
      {
        assert(CopSSHull.p.size()==4);
        copSSHull_=CopSSHull;
      }



      /// \brief Get the state of the Foot along the X coordinate
      ///        It is a vector of size 4:
      ///        (Position, Velocity, Acceleration, 1)
      inline void setStateX(const VectorX& state)
      {
        stateX_=state;
      }

      /// \brief Get the state of the Foot along the X coordinate
      inline const VectorX& getStateX() const
      {return stateX_;}

      /// \brief Get the state of the Foot along the Y coordinate
      ///        It is a vector of size 4:
      ///        (Position, Velocity, Acceleration, 1)
      inline void setStateY(const VectorX& state)
      {
        stateY_=state;
      }

      /// \brief Get the state of the Foot along the Y coordinate
      inline const VectorX& getStateY() const
      {return stateY_;}

      /// \brief Get the state of the Foot along the Z coordinate
      ///        It is a vector of size 4:
      ///        (Position, Velocity, Acceleration, 1)
      inline void setStateZ(const VectorX& state)
      {
        stateZ_=state;
      }

      /// \brief Get the state of the Foot along the Z coordinate
      inline const VectorX& getStateZ() const
      {return stateZ_;}

      /// \brief Get the state of the Foot around Yaw axis
      ///        It is a vector of size 4:
      ///        (Position, Velocity, Acceleration, 1)
      inline void setStateYaw(const VectorX& state)
      {
        stateYaw_=state;
      }

      /// \brief Get the state of the Foot around Yaw axis
      inline const VectorX& getStateYaw() const
      {return stateYaw_;}

      /// \brief True if the foot touch the ground at the ith sample
      inline bool isInContact(int i) const
      {
        assert(i>=0);
        return isInContact_[i];
      }

      /// \brief True if the foot is the support foot  at the ith sample
      inline bool isSupportFoot(int i) const
      {
        assert(i>=0);
        return isSupportFoot_[i];
      }

      /// \brief Compute the selection matrix attached to the foot
      void computeSelectionMatrix();

      // Delete it? Yes if SelectionMatrix are always used after being casted
      // into linearDynamic
      /// \brief Get the selection matrix attached to the foot
      inline const SelectionMatrix& getSelectionMatrix() const
      {return selectionMatrix_;}

      /// \brief Compute the foot position dynamic matrices
      void computeFootPosDynamic();

      /// \brief Get the linear dynamic corresponding to the foot position
      inline const LinearDynamic& getFootPosLinearDynamic() const
      {return footPosDynamic_;}

      /// \brief Compute the rotation matrix attached to the foot.
      ///        If the foot is not the current support foot for a given
      ///        sample, the corresponding element is set to zero.
      void computeRotationMatrix();

      /// \brief Get the rotation matrix attached to the foot
      inline const MatrixX& getRotationMatrix() const
      {return rotationMatrix_;}

      /// \brief Get the transpose of the rotation matrix attached to the foot
      inline const MatrixX& getRotationMatrixT() const
      {return rotationMatrixT_;}

      /// \brief Setters for the kinematic limits of the foot
      void setHipYawUpperBound(
          Scalar hipYawUpperBound);
      void setHipYawSpeedUpperBound(
          Scalar hipYawSpeedUpperBound);
      void setHipYawAccelerationUpperBound(
          Scalar hipYawAccelerationUpperBound);
      void setHipYawLowerBound(
          Scalar hipYawLowerBound);
      void setMaxHeight(
          Scalar maxHeight);

      /// \brief Set the hull of the reachable positions by the foot
      void setKinematicHull(const Hull& kinematicHull);
      /// \brief Get the hull of the reachable positions by the foot
      inline const Hull& getKinematicHull() const
      {return kinematicLimits_.kinematicHull_;}

    private:
      /// \brief constructors initialization instruction
      void xInit();
      /// \brief default initialization of SS and DS CoP hulls
      void xCreateDefaultCopHulls();

    private:
      int nbSamples_;
      Scalar samplingPeriod_;
      int nbPreviewedSteps_;

      VectorX stateX_;
      VectorX stateY_;
      VectorX stateZ_;
      VectorX stateYaw_;

      SelectionMatrix selectionMatrix_;
      LinearDynamic footPosDynamic_;

      MatrixX rotationMatrix_;
      MatrixX rotationMatrixT_;

      KinematicLimits kinematicLimits_;

      Hull copDSHull_;
      Hull copSSHull_;

      /// \brief Vector of size nbSamples_
      std::vector<bool> isInContact_;
      /// \brief Vector of size nbPreviewedSteps_
      std::vector<bool> isSupportFoot_;
  };
}

#endif // MPC_WALKGEN_HUMANOID_FOOT_MODEL_H
