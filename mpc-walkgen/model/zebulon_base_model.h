////////////////////////////////////////////////////////////////////////////////
///
///\file base_model.h
///\brief Implement the zebulon base model
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_MODEL_ZEBULON_BASE_MODEL_H
#define MPC_WALKGEN_MODEL_ZEBULON_BASE_MODEL_H

#include <mpc-walkgen/convexpolygon.h>
#include <mpc-walkgen/lineardynamic.h>

#ifdef _MSC_VER
# pragma warning( push )
// C4251: class needs to have DLL interface
# pragma warning( disable: 4251 )
#endif

namespace MPCWalkgen
{
  template <typename Scalar>
  class MPC_WALKGEN_API BaseModel
  {
    TEMPLATE_TYPEDEF(Scalar)

    public:
      BaseModel(int nbSamples,
                Scalar samplingPeriod,
                bool autoCompute);
      BaseModel();
      ~BaseModel();

    /// \brief compute all of the dynamics
    void computeDynamics();
    void computeCopXDynamic();
    void computeCopYDynamic();
    void computeBasePosDynamic();
    void computeBaseVelDynamic();
    void computeBaseAccDynamic();
    void computeBaseJerkDynamic();
    void computeTiltDynamic();

    /// \brief Get the linear dynamic correspond to the CoP position X
    inline const LinearDynamic<Scalar>& getCopXLinearDynamic() const
    {return copXDynamic_;}

    /// \brief Get the linear dynamic correspond to the CoP position Y
    inline const LinearDynamic<Scalar>& getCopYLinearDynamic() const
    {return copYDynamic_;}

    /// \brief Get the linear dynamic correspond to the base position
    inline const LinearDynamic<Scalar>& getBasePosLinearDynamic() const
    {return basePosDynamic_;}

    /// \brief Get the linear dynamic correspond to the base tilt angle
    inline const LinearDynamic<Scalar>& getBaseTiltAngleLinearDynamic() const
    {return baseTiltAngleDynamic_;}

    /// \brief Get the linear dynamic correspond to the base tilt velocity
    inline const LinearDynamic<Scalar>& getBaseTiltAngularVelLinearDynamic() const
    {return baseTiltAngularVelDynamic_;}

    /// \brief Get the linear dynamic correspond to the base velocity
    inline const LinearDynamic<Scalar>& getBaseVelLinearDynamic() const
    {return baseVelDynamic_;}

    /// \brief Get the linear dynamic correspond to the base acceleration
    inline const LinearDynamic<Scalar>& getBaseAccLinearDynamic() const
    {return baseAccDynamic_;}

    /// \brief Get the linear dynamic correspond to the base jerk
    inline const LinearDynamic<Scalar>& getBaseJerkLinearDynamic() const
    {return baseJerkDynamic_;}

    /// \brief Get the state of the base along the X coordinate
    inline const VectorX& getStateX() const
    {return stateX_;}

    /// \brief Get the state of the CoM along the X coordinate
    ///        It's a vector of size 4:
    ///        (Position, Velocity, Acceleration, 1)
    inline void setStateX(const VectorX& state)
    {
      assert(state==state);
      assert(state.size()==3);
      stateX_ = state;
    }


    /// \brief Get the state of the base along the Y coordinate
    inline const VectorX& getStateY() const
    {return stateY_;}

    /// \brief Get the state of the CoM along the Y coordinate
    ///        It's a vector of size 4:
    ///        (Position, Velocity, Acceleration, 1)
    inline void setStateY(const VectorX& state)
    {
      assert(state==state);
      assert(state.size()==3);
      stateY_ = state;
    }


    /// \brief Get the state of the base along the Theta coordinate
    inline const VectorX& getStateRoll() const
    {return stateRoll_;}

    /// \brief Get the state of the CoM along the Theta coordinate
    ///        It's a vector of size 4:
    ///        (Position, Velocity, Acceleration, 1)
    inline void setStateRoll(const VectorX& state)
    {
      assert(state==state);
      assert(state.size()==3);
      stateRoll_ = state;
    }

    /// \brief Get the state of the base along the Theta coordinate
    inline const VectorX& getStatePitch() const
    {return statePitch_;}

    /// \brief Get the state of the CoM along the Theta coordinate
    ///        It's a vector of size 4:
    ///        (Position, Velocity, Acceleration, 1)
    inline void setStatePitch(const VectorX& state)
    {
      assert(state==state);
      assert(state.size()==3);
      statePitch_ = state;
    }

    /// \brief Get the state of the base along the Theta coordinate
    inline const VectorX& getStateYaw() const
    {return stateYaw_;}

    /// \brief Get the state of the CoM along the Theta coordinate
    ///        It's a vector of size 4:
    ///        (Position, Velocity, Acceleration, 1)
    inline void setStateYaw(const VectorX& state)
    {
      assert(state==state);
      assert(state.size()==3);
      stateYaw_ = state;
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

    /// \brief Set the CoM constant height
    void setComHeight(Scalar comHeight);

    /// \brief Get the CoM constant height
    inline Scalar getComHeight(void) const
    {return comHeight_;}

    /// \brief Set the base mass
    void setMass(Scalar mass);

    /// \brief Get the base mass
    inline Scalar getMass(void) const
    {return mass_;}

    /// \brief Set the robot total mass
    void setTotalMass(Scalar mass);

    /// \brief Set the constant gravity vector
    void setGravity(const Vector3& gravity);

    /// \brief Get the base velocity maximum value
    inline Scalar getVelocityLimit(void) const
    {return velocityLimit_;}

    /// \brief Set the base velocity maximum value
    inline void setVelocityLimit(Scalar velocityLimit)
    {
      assert(velocityLimit>=0);
      velocityLimit_ = velocityLimit;
    }

    /// \brief Get the base acceleration maximum value
    inline Scalar getAccelerationLimit(void) const
    {return accelerationLimit_;}

    /// \brief Set the base acceleration maximum value
    inline void setAccelerationLimit(Scalar accelerationLimit)
    {
      assert(accelerationLimit>=0);
      accelerationLimit_ = accelerationLimit;
    }

    /// \brief Get the base jerk maximum value
    inline Scalar getJerkLimit(void) const
    {return jerkLimit_;}

    /// \brief Set the base jerk maximum value
    inline void setJerkLimit(Scalar jerkLimit)
    {
      assert(jerkLimit>=0);
      jerkLimit_ = jerkLimit;
    }


    /// \brief Get the base support polygon
    inline const ConvexPolygon<Scalar>& getCopSupportConvexPolygon(void) const
    {return copSupportConvexPolygon_;}

    /// \brief Set the base support polygon
    inline void setCopSupportConvexPolygon(const ConvexPolygon<Scalar>& supportConvexPolygon)
    {
      assert(supportConvexPolygon.getNbVertices()>=3);
      copSupportConvexPolygon_ = supportConvexPolygon;
    }

    /// \brief Get the base support polygon
    inline const ConvexPolygon<Scalar>& getComSupportConvexPolygon(void) const
    {return comSupportConvexPolygon_;}

    /// \brief Set the base support polygon
    inline void setComSupportConvexPolygon(const ConvexPolygon<Scalar>& supportConvexPolygon)
    {
      assert(supportConvexPolygon.getNbVertices()>=3);
      comSupportConvexPolygon_ = supportConvexPolygon;
    }

    inline void setTiltContactPointX(Scalar pos)
    {
      tiltContactPointX_ = pos;
    }
    inline Scalar getTiltContactPointX() const
    {
      return tiltContactPointX_;
    }

    inline void setTiltContactPointY(Scalar pos)
    {
      tiltContactPointY_ = pos;
    }
    inline Scalar getTiltContactPointY() const
    {
      return tiltContactPointY_;
    }

  private:
    bool autoCompute_;

    int nbSamples_;
    Scalar samplingPeriod_;

    VectorX stateX_;
    VectorX stateY_;
    VectorX stateRoll_;
    VectorX statePitch_;
    VectorX stateYaw_;


    Scalar comHeight_;
    Vector3 gravity_;
    Scalar mass_;
    Scalar totalMass_;

    Scalar velocityLimit_;
    Scalar accelerationLimit_;
    Scalar jerkLimit_;

    Scalar tiltContactPointX_;
    Scalar tiltContactPointY_;

    LinearDynamic<Scalar> basePosDynamic_;
    LinearDynamic<Scalar> baseVelDynamic_;
    LinearDynamic<Scalar> baseAccDynamic_;
    LinearDynamic<Scalar> baseJerkDynamic_;
    LinearDynamic<Scalar> copXDynamic_;
    LinearDynamic<Scalar> copYDynamic_;
    LinearDynamic<Scalar> baseTiltAngleDynamic_;
    LinearDynamic<Scalar> baseTiltAngularVelDynamic_;

    ConvexPolygon<Scalar> copSupportConvexPolygon_;
    ConvexPolygon<Scalar> comSupportConvexPolygon_;
  };
}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
