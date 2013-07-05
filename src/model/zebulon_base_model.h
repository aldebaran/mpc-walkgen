////////////////////////////////////////////////////////////////////////////////
///
///\file	base_model.h
///\brief	Implement the zebulon base model
///\author Lafaye Jory
///\version	1.0
///\date	19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_ZEBULON_BASE_MODEL_H
#define MPC_WALKGEN_ZEBULON_BASE_MODEL_H

#include "../type.h"

namespace MPCWalkgen
{

  class BaseModel
  {
  public:
    BaseModel(int nbSamples = 1,
              Scalar samplingPeriod = 1.0,
              Scalar velocityLimit = 1.0,
              Scalar accelerationLimit = 1.0,
              Scalar jerkLimit = 1.0,
              const Hull& supportHull = Hull(),
              bool autoCompute = true);
    ~BaseModel();

    /// \brief compute all of the dynamics
    void computeDynamics();
    void computeBasePosDynamic();
    void computeBaseVelDynamic();
    void computeBaseAccDynamic();
    void computeBaseJerkDynamic();

    /// \brief Get the linear dynamic correspond to the base position
    inline const LinearDynamic& getBasePosLinearDynamic() const
    {return basePosDynamic_;}

    /// \brief Get the linear dynamic correspond to the base velocity
    inline const LinearDynamic& getBaseVelLinearDynamic() const
    {return baseVelDynamic_;}

    /// \brief Get the linear dynamic correspond to the base acceleration
    inline const LinearDynamic& getBaseAccLinearDynamic() const
    {return baseAccDynamic_;}

    /// \brief Get the linear dynamic correspond to the base jerk
    inline const LinearDynamic& getBaseJerkLinearDynamic() const
    {return baseJerkDynamic_;}

    /// \brief Get the state of the base along the X coordinate
    inline const VectorX& getStateX() const
    {return stateX_;}

    /// \brief Get the state of the CoM along the X coordinate
    ///        It's a vector of size 4:
    ///        (Position, Velocity, Acceleration, 1)
    inline void setStateX(const VectorX& state)
    {
      assert(state==state);assert(state.size()==4);assert(state(3)==1);
      stateX_=state;
    }


    /// \brief Get the state of the base along the Y coordinate
    inline const VectorX& getStateY() const
    {return stateY_;}

    /// \brief Get the state of the CoM along the Y coordinate
    ///        It's a vector of size 4:
    ///        (Position, Velocity, Acceleration, 1)
    inline void setStateY(const VectorX& state)
    {
      assert(state==state);assert(state.size()==4);assert(state(3)==1);
      stateY_=state;
    }


    /// \brief Set the number of samples for this dynamic
    void setNbSamples(int nbSamples);

    /// \brief Get the number of samples for this dynamic
    inline int getNbSamples() const
    {return nbSamples_;}

    /// \brief Update the X state of the CoM apply a constant jerk
    void updateStateX(Scalar jerk, Scalar feedBackPeriod);

    /// \brief Update the Y state of the CoM apply a constant jerk
    void updateStateY(Scalar jerk, Scalar feedBackPeriod);

    /// \brief Set the sampling period for each sample
    void setSamplingPeriod(Scalar samplingPeriod);

    /// \brief Get the sampling period for each sample
    inline Scalar getSamplingPeriod(void) const
    {return samplingPeriod_;}

    /// \brief Get the base velocity maximum value
    inline Scalar getVelocityLimit(void) const
    {return velocityLimit_;}

    /// \brief Set the base velocity maximum value
    inline void setVelocityLimit(Scalar velocityLimit)
    {
      assert(velocityLimit>=0);
      velocityLimit_=velocityLimit;
    }

    /// \brief Get the base acceleration maximum value
    inline Scalar getAccelerationLimit(void) const
    {return accelerationLimit_;}

    /// \brief Set the base acceleration maximum value
    inline void setAccelerationLimit(Scalar accelerationLimit)
    {
      assert(accelerationLimit>=0);
      accelerationLimit_=accelerationLimit;
    }

    /// \brief Get the base jerk maximum value
    inline Scalar getJerkLimit(void) const
    {return jerkLimit_;}

    /// \brief Set the base jerk maximum value
    inline void setJerkLimit(Scalar jerkLimit)
    {
      assert(jerkLimit>=0);
      jerkLimit_=jerkLimit;
    }


    /// \brief Get the base support polygon
    inline const Hull& getSupportHull(void) const
    {return supportHull_;}

    /// \brief Set the base support polygon
    inline void setSupportHull(const Hull& supportHull)
    {
      assert(supportHull.p.size()>=3);
      supportHull_=supportHull;
    }



  private:
    bool autoCompute_;

    int nbSamples_;
    Scalar samplingPeriod_;

    VectorX stateX_;
    VectorX stateY_;

    Scalar velocityLimit_;
    Scalar accelerationLimit_;
    Scalar jerkLimit_;

    LinearDynamic basePosDynamic_;
    LinearDynamic baseVelDynamic_;
    LinearDynamic baseAccDynamic_;
    LinearDynamic baseJerkDynamic_;
    Hull supportHull_;

  };

}

#endif //MPC_WALKGEN_ZEBULON_BASE_MODEL_H
