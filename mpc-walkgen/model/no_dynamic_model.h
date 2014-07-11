////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_NO_DYNAMIC_MODEL_H
#define MPC_WALKGEN_NO_DYNAMIC_MODEL_H

#include <mpc-walkgen/lineardynamic.h>

#ifdef _MSC_VER
# pragma warning( push )
// C4251: class needs to have DLL interface
# pragma warning( disable: 4251 )
#endif

namespace MPCWalkgen
{
  template <typename Scalar>
  class MPC_WALKGEN_API NoDynamicModel
  {
    TEMPLATE_TYPEDEF(Scalar)

    public:
      NoDynamicModel(int nbSamples,
                Scalar samplingPeriod,
                bool autoCompute);
      NoDynamicModel();
      ~NoDynamicModel();

    /// \brief compute all of the dynamics
    void computeDynamics();
    void computePosDynamic();
    void computeVelDynamic();
    void computeAccDynamic();
    void computeJerkDynamic();

    /// \brief Get the linear dynamic correspond to the base position
    inline const LinearDynamic<Scalar>& getPosLinearDynamic() const
    {return posDynamic_;}

    /// \brief Get the linear dynamic correspond to the base velocity
    inline const LinearDynamic<Scalar>& getVelLinearDynamic() const
    {return velDynamic_;}

    /// \brief Get the linear dynamic correspond to the base acceleration
    inline const LinearDynamic<Scalar>& getAccLinearDynamic() const
    {return accDynamic_;}

    /// \brief Get the linear dynamic correspond to the base jerk
    inline const LinearDynamic<Scalar>& getJerkLinearDynamic() const
    {return jerkDynamic_;}

    /// \brief Get the state of the base
    inline const VectorX& getState() const
    {return state_;}

    /// \brief Get the state
    ///        It's a vector of size 3:
    ///        (Position, Velocity, Acceleration)
    inline void setState(const VectorX& state)
    {
      assert(state==state);
      assert(state.size()==3);
      state_ = state;
    }

    /// \brief Set the number of samples for this dynamic
    void setNbSamples(int nbSamples);

    /// \brief Get the number of samples for this dynamic
    inline int getNbSamples() const
    {return nbSamples_;}

    /// \brief Update the state
    void updateState(Scalar jerk, Scalar feedBackPeriod);

    /// \brief Set the sampling period for each sample
    void setSamplingPeriod(Scalar samplingPeriod);

    /// \brief Get the sampling period for each sample
    inline Scalar getSamplingPeriod(void) const
    {return samplingPeriod_;}

    /// \brief Get the velocity maximum value
    inline Scalar getVelocityLimit(void) const
    {return velocityLimit_;}

    /// \brief Set the velocity maximum value
    inline void setVelocityLimit(Scalar velocityLimit)
    {
      assert(velocityLimit>=0);
      velocityLimit_ = velocityLimit;
    }

    /// \brief Get the acceleration maximum value
    inline Scalar getAccelerationLimit(void) const
    {return accelerationLimit_;}

    /// \brief Set the acceleration maximum value
    inline void setAccelerationLimit(Scalar accelerationLimit)
    {
      assert(accelerationLimit>=0);
      accelerationLimit_ = accelerationLimit;
    }

    /// \brief Get the jerk maximum value
    inline Scalar getJerkLimit(void) const
    {return jerkLimit_;}

    /// \brief Set the jerk maximum value
    inline void setJerkLimit(Scalar jerkLimit)
    {
      assert(jerkLimit>=0);
      jerkLimit_ = jerkLimit;
    }

  private:
    bool autoCompute_;

    int nbSamples_;
    Scalar samplingPeriod_;

    VectorX state_;

    Scalar velocityLimit_;
    Scalar accelerationLimit_;
    Scalar jerkLimit_;

    LinearDynamic<Scalar> posDynamic_;
    LinearDynamic<Scalar> velDynamic_;
    LinearDynamic<Scalar> accDynamic_;
    LinearDynamic<Scalar> jerkDynamic_;
  };
}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
