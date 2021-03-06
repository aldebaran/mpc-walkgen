////////////////////////////////////////////////////////////////////////////////
///
///\file lip_model.h
///\brief Implement the linear inverted pendulum model
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_MODEL_LIPMODEL_H
#define MPC_WALKGEN_MODEL_LIPMODEL_H

#include <mpc-walkgen/lineardynamic.h>
#include <vector>

#ifdef _MSC_VER
# pragma warning( push )
// C4251: class needs to have DLL interface
# pragma warning( disable: 4251 )
#endif

namespace MPCWalkgen
{
  template <typename Scalar>
  class MPC_WALKGEN_API LIPModel
  {
    TEMPLATE_TYPEDEF(Scalar)

    public:

      LIPModel(int nbSamples,
               Scalar samplingPeriod,
               bool autoCompute);
      LIPModel();
      ~LIPModel();

      /// \brief Compute all dynamics related to the LIP model.
      ///        In humanoid,
      void computeDynamics();

      void computeCopXDynamicVec();
      void computeCopYDynamicVec();
      void computeComPosDynamicVec();
      void computeComVelDynamicVec();
      void computeComAccDynamicVec();
      void computeComJerkDynamic();

      /// \brief Get the linear dynamic correspond to the CoP position X
      inline const LinearDynamic<Scalar>& getCopXLinearDynamic() const
      {return copXDynamicVec_[nbFeedbackInOneSample_ - 1];}

      /// \brief Get the linear dynamic correspond to the CoP position Y
      inline const LinearDynamic<Scalar>& getCopYLinearDynamic() const
      {return copYDynamicVec_[nbFeedbackInOneSample_ - 1];}

      /// \brief Get the linear dynamic correspond to the CoM position
      inline const LinearDynamic<Scalar>& getComPosLinearDynamic() const
      {return comPosDynamicVec_[nbFeedbackInOneSample_ - 1];}

      /// \brief Get the linear dynamic correspond to the CoM velocity
      inline const LinearDynamic<Scalar>& getComVelLinearDynamic() const
      {return comVelDynamicVec_[nbFeedbackInOneSample_ - 1];}

      /// \brief Get the linear dynamic correspond to the CoM acceleration
      inline const LinearDynamic<Scalar>& getComAccLinearDynamic() const
      {return comAccDynamicVec_[nbFeedbackInOneSample_ - 1];}


      /// \brief Get the linear dynamic correspond to the CoM jerk
      inline const LinearDynamic<Scalar>& getComJerkLinearDynamic() const
      {return comJerkDynamic_;}

      /// \brief Those function also return the dynamics of the LIP model, but they allow
      ///        to be synchronized with a timeline

      inline const LinearDynamic<Scalar>& getCopXLinearDynamic(int index) const
      {
        assert(index<nbFeedbackInOneSample_);
        return copXDynamicVec_[index];
      }

      inline const LinearDynamic<Scalar>& getCopYLinearDynamic(int index) const
      {
        assert(index<nbFeedbackInOneSample_);
        return copYDynamicVec_[index];
      }

      inline const LinearDynamic<Scalar>& getComPosLinearDynamic(int index) const
      {
        assert(index<nbFeedbackInOneSample_);
        return comPosDynamicVec_[index];
      }

      inline const LinearDynamic<Scalar>& getComVelLinearDynamic(int index) const
      {
        assert(index<nbFeedbackInOneSample_);
        return comVelDynamicVec_[index];
      }

      inline const LinearDynamic<Scalar>& getComAccLinearDynamic(int index) const
      {
        assert(index<nbFeedbackInOneSample_);
        return comAccDynamicVec_[index];
      }

      inline const LinearDynamic<Scalar>& getComJerkLinearDynamic(int index) const
      {
        assert(index<nbFeedbackInOneSample_);
        return comJerkDynamic_;
      }


      /// \brief Get the state of the CoM along the X coordinate
      ///        It is a vector of size 3:
      ///        (Position, Velocity, Acceleration)
      inline void setStateX(const VectorX& state)
      {
        assert(state==state);
        assert(state.size()==3);
        stateX_=state;
      }

      /// \brief Get the state of the CoM along the X coordinate
      inline const VectorX& getStateX() const
      {return stateX_;}

      /// \brief Get the state of the CoM along the Y coordinate
      ///        It is a vector of size 3:
      ///        (Position, Velocity, Acceleration)
      inline void setStateY(const VectorX& state)
      {
        assert(state==state);
        assert(state.size()==3);
        stateY_=state;
      }

      /// \brief Get the state of the CoM along the Y coordinate
      inline const VectorX& getStateY() const
      {return stateY_;}

      /// \brief Get the state of the CoM along the Z coordinate
      inline const VectorX& getStateZ() const
      {return stateZ_;}

      /// \brief Get the yaw state of the CoM around the Z axis
      ///        It is a vector of size 3:
      ///        (Position, Velocity, Acceleration)
      inline void setStateYaw(const VectorX& state)
      {
        assert(state==state);
        assert(state.size()==3);

        stateYaw_=state;
      }

      /// \brief Get the state of the CoM around the Z coordinate
      inline const VectorX& getStateYaw() const
      {return stateYaw_;}

      /// \brief Set the number of samples for this dynamic
      void setNbSamples(int nbSamples);

      /// \brief Get the number of samples for this dynamic
      inline int getNbSamples() const
      {return nbSamples_;}

      /// \brief Update the X state of the CoM apply a constant jerk
      void updateStateX(Scalar jerk, Scalar feedBackPeriod);

      /// \brief Update the Y state of the CoM apply a constant jerk
      void updateStateY(Scalar jerk, Scalar feedBackPeriod);

      /// \brief Update the Yaw state of the CoM considering a constant angular jerk
      void updateStateYaw(Scalar jerk, Scalar feedBackPeriod);

      /// \brief Set the sampling period for each sample
      void setSamplingPeriod(Scalar samplingPeriod);

      /// \brief Set the feedback period of the MPC that uses this LIP model
      void setFeedbackPeriod(Scalar feedbackPeriod);

      /// \brief Get the sampling period for each sample
      inline Scalar getSamplingPeriod(void) const
      {return samplingPeriod_;}

      /// \brief Get the feedback period of the MPC that uses this LIP model
      inline Scalar getFeedbackPeriod() const
      {return feedbackPeriod_;}

      /// \brief Set the CoM constant height
      void setComHeight(Scalar comHeight);

      /// \brief Get the CoM constant height
      inline Scalar getComHeight(void) const
      {return comHeight_;}

      /// \brief Set the body mass
      void setMass(Scalar mass);

      /// \brief Get the body mass
      inline Scalar getMass(void) const
      {return mass_;}

      /// \brief Set the robot total mass
      void setTotalMass(Scalar mass);

      /// \brief Set the constant gravity vector
      void setGravity(const Vector3& gravity);

      /// \brief Get the constant gravity vector
      inline const Vector3& getGravity(void) const
      {return gravity_;}

    private:
      bool autoCompute_;

      bool useLipModel2_;

      int nbSamples_;
      Scalar samplingPeriod_;
      Scalar feedbackPeriod_;
      int nbFeedbackInOneSample_;

      VectorX stateX_;
      VectorX stateY_;
      VectorX stateZ_;
      VectorX stateYaw_;

      Scalar comHeight_;
      Vector3 gravity_;
      Scalar mass_;
      Scalar totalMass_;

      LinearDynamic<Scalar> comPosDynamic_;
      LinearDynamic<Scalar> comVelDynamic_;
      LinearDynamic<Scalar> comAccDynamic_;
      LinearDynamic<Scalar> copXDynamic_;
      LinearDynamic<Scalar> copYDynamic_;

      std::vector<LinearDynamic<Scalar> > comPosDynamicVec_;
      std::vector<LinearDynamic<Scalar> > comVelDynamicVec_;
      std::vector<LinearDynamic<Scalar> > comAccDynamicVec_;
      std::vector<LinearDynamic<Scalar> > copXDynamicVec_;
      std::vector<LinearDynamic<Scalar> > copYDynamicVec_;

      LinearDynamic<Scalar> comJerkDynamic_;
  };

}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
