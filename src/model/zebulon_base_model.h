////////////////////////////////////////////////////////////////////////////////
///
///\file base_model.h
///\brief Implement the zebulon base model
///\author Lafaye Jory
///\date 19/06/13
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

    /// \brief Get the linear dynamic correspond to the CoP position X
    inline const LinearDynamic& getCopXLinearDynamic() const
    {return copXDynamic_;}


    /// \brief Get the linear dynamic correspond to the CoP position Y
    inline const LinearDynamic& getCopYLinearDynamic() const
    {return copYDynamic_;}

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
      assert(state==state);assert(state.size()==4);assert(state(3)==1.0);
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
      assert(state==state);assert(state.size()==4);assert(state(3)==1.0);
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
      assert(state==state);assert(state.size()==4);assert(state(3)==1.0);
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
      assert(state==state);assert(state.size()==4);assert(state(3)==1.0);
      statePitch_ = state;
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
    inline const ConvexPolygon& getCopSupportConvexPolygon(void) const
    {return copSupportConvexPolygon_;}

    /// \brief Set the base support polygon
    inline void setCopSupportConvexPolygon(const ConvexPolygon& supportConvexPolygon)
    {
      assert(supportConvexPolygon.getNbVertices()>=3);
      copSupportConvexPolygon_ = supportConvexPolygon;
    }

    /// \brief Get the base support polygon
    inline const ConvexPolygon& getComSupportConvexPolygon(void) const
    {return comSupportConvexPolygon_;}

    /// \brief Set the base support polygon
    inline void setComSupportConvexPolygon(const ConvexPolygon& supportConvexPolygon)
    {
      assert(supportConvexPolygon.getNbVertices()>=3);
      comSupportConvexPolygon_ = supportConvexPolygon;
    }

    inline Scalar getWheelToBaseDistance(void) const
    {return wheelToBaseDist_;}

    inline void setWheelToBaseDistance(Scalar dist)
    {
      assert(dist>=0.0);
      wheelToBaseDist_ = dist;
    }

    inline Scalar getAngleWheelToBaseCom(void) const
    {return angleWheelToBaseCom_;}

    inline void setAngleWheelToBaseCom(Scalar angle)
    {
      angleWheelToBaseCom_ = angle;
    }

  private:
    bool autoCompute_;

    int nbSamples_;
    Scalar samplingPeriod_;

    VectorX stateX_;
    VectorX stateY_;
    VectorX stateRoll_;
    VectorX statePitch_;

    Scalar comHeight_;
    Vector3 gravity_;
    Scalar mass_;
    Scalar totalMass_;

    Scalar velocityLimit_;
    Scalar accelerationLimit_;
    Scalar jerkLimit_;

    LinearDynamic basePosDynamic_;
    LinearDynamic baseVelDynamic_;
    LinearDynamic baseAccDynamic_;
    LinearDynamic baseJerkDynamic_;
    LinearDynamic copXDynamic_;
    LinearDynamic copYDynamic_;
    ConvexPolygon copSupportConvexPolygon_;
    ConvexPolygon comSupportConvexPolygon_;

    Scalar wheelToBaseDist_;
    Scalar angleWheelToBaseCom_;

  };

}

#endif //MPC_WALKGEN_ZEBULON_BASE_MODEL_H
