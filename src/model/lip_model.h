////////////////////////////////////////////////////////////////////////////////
///
///\file	lip_model.h
///\brief	Implement the linear inverted pendulum model
///\author Lafaye Jory
///\version	1.0
///\date	19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_LIPM_H
#define MPC_WALKGEN_LIPM_H

#include "../type.h"

namespace MPCWalkgen
{

  class LIPModel
  {
  public:
    LIPModel(int nbSamples = 1,
             Scalar samplingPeriod = 1.0,
             Scalar comHeight = 1.0,
             Vector3 gravity = Vector3(0.0, 0.0, 9.81),
             bool autoCompute = true);

    ~LIPModel();

    /// \brief compute all of the dynamics
    void computeDynamics();
    void computeCopXDynamic();
    void computeCopYDynamic();
    void computeComPosDynamic();
    void computeComVelDynamic();
    void computeComAccDynamic();
    void computeComJerkDynamic();

    /// \brief Get the linear dynamic correspond to the CoP position X
    inline const LinearDynamic& getCopXLinearDynamic() const
    {return copXDynamic_;}


    /// \brief Get the linear dynamic correspond to the CoP position Y
    inline const LinearDynamic& getCopYLinearDynamic() const
    {return copYDynamic_;}

    /// \brief Get the linear dynamic correspond to the CoM position
    inline const LinearDynamic& getComPosLinearDynamic() const
    {return comPosDynamic_;}

    /// \brief Get the linear dynamic correspond to the CoM velocity
    inline const LinearDynamic& getComVelLinearDynamic() const
    {return comVelDynamic_;}


    /// \brief Get the linear dynamic correspond to the CoM acceleration
    inline const LinearDynamic& getComAccLinearDynamic() const
    {return comAccDynamic_;}


    /// \brief Get the linear dynamic correspond to the CoM jerk
    inline const LinearDynamic& getComJerkLinearDynamic() const
    {return comJerkDynamic_;}

    /// \brief Get the state of the CoM along the X coordinate
    ///        It's a vector of size 4:
    ///        (Position, Velocity, Acceleration, 1)
    inline void setStateX(const VectorX& state)
    {
      assert(state==state);assert(state.size()==4);assert(state(3)==1);
      stateX_=state;
    }

    /// \brief Get the state of the base along the X coordinate
    inline const VectorX& getStateX() const
    {return stateX_;}

    /// \brief Get the state of the CoM along the Y coordinate
    ///        It's a vector of size 4:
    ///        (Position, Velocity, Acceleration, 1)
    inline void setStateY(const VectorX& state)
    {
      assert(state==state);assert(state.size()==4);assert(state(3)==1);
      stateY_=state;
    }

    /// \brief Get the state of the base along the Y coordinate
    inline const VectorX& getStateY() const
    {return stateY_;}

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

    /// \brief Set the CoM constant height
    void setComHeight(Scalar comHeight);

    /// \brief Set the constant gravity vector
    void setGravity(const Vector3& gravity);

  private:
    bool autoCompute_;

    int nbSamples_;
    Scalar samplingPeriod_;

    VectorX stateX_;
    VectorX stateY_;

    Scalar comHeight_;
    Vector3 gravity_;

    LinearDynamic comPosDynamic_;
    LinearDynamic comVelDynamic_;
    LinearDynamic comAccDynamic_;
    LinearDynamic comJerkDynamic_;
    LinearDynamic copXDynamic_;
    LinearDynamic copYDynamic_;
  };

}

#endif //MPC_WALKGEN_LIPM_H
