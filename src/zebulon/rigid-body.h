#pragma once
#ifndef MPC_WALKGEN_ZEBULON_RIGID_BODY_H
#define MPC_WALKGEN_ZEBULON_RIGID_BODY_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	rigid-body.h
///\brief	A class to store rigid bodies
///\author	Lafaye Jory
///\version	1.0
///\date	02/05/12
///
////////////////////////////////////////////////////////////////////////////////


#include "types.h"
#include "../common/interpolation.h"


namespace MPCWalkgen{
  namespace Zebulon{
    class RigidBody{

    public:
      RigidBody(const MPCData * generalData,
                const RobotData * robotData,
                const Interpolation * interpolation);
      virtual ~RigidBody();

      inline const BodyState & state() const{return state_;}
      inline BodyState & state(){return state_;}

      inline void state(const BodyState & s){state_=s;}

      const LinearDynamics & dynamics(DynamicMatrixType type) const;

      void computeQPDynamics();

      void computeInterpolationDynamics();

      void computeQPDynamicsCoP();

      void computeInterpolationDynamicsCoP();

      virtual void interpolate(GlobalSolution & result, double currentTime, const Reference & velRef)=0;

    protected:
      virtual void computeDynamicsMatrices(LinearDynamics & dyn,
                                           double S, double T, int N, DynamicMatrixType type)=0;


    protected:
      const MPCData *generalData_;
      const RobotData *robotData_;
      const Interpolation *interpolation_;

      BodyState state_;
      LinearDynamics posInt_vec_;
      LinearDynamics pos_vec_;
      LinearDynamics vel_vec_;
      LinearDynamics acc_vec_;
      LinearDynamics jerk_vec_;
      LinearDynamics copX_vec_;
      LinearDynamics copY_vec_;

      LinearDynamics posIntInterpol_;
      LinearDynamics posInterpol_;
      LinearDynamics velInterpol_;
      LinearDynamics accInterpol_;
      LinearDynamics copXInterpol_;
      LinearDynamics copYInterpol_;

      int matrixNumber_;
    };
  }
}

#endif // MPC_WALKGEN_ZEBULON_MPC_WALKGEN_ZEBULON_RIGID_BODY_H_H
