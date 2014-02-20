////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_walkgen.h
///\brief Main program for Humanoid
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_HUMANOID_WALKGEN_H
#define MPC_WALKGEN_HUMANOID_WALKGEN_H

#include <mpc-walkgen/api.h>
#include <mpc-walkgen/type.h>
#include <vector>
#include <mpc-walkgen/convexpolygon.h>
#include <mpc-walkgen/model/lip_model.h>
#include <mpc-walkgen/humanoid_feet_supervisor.h>
#include <mpc-walkgen/humanoid_walkgen_type.h>
#include <mpc-walkgen/function/humanoid_cop_centering_objective.h>
#include <mpc-walkgen/function/humanoid_cop_constraint.h>
#include <mpc-walkgen/function/humanoid_foot_constraint.h>
#include <mpc-walkgen/function/humanoid_lip_com_jerk_minimization_objective.h>
#include <mpc-walkgen/function/humanoid_lip_com_velocity_tracking_objective.h>
#include <mpc-walkgen/function/zebulon_base_motion_constraint.h>
#include <mpc-walkgen/function/zebulon_base_position_tracking_objective.h>
#include <mpc-walkgen/function/zebulon_base_velocity_tracking_objective.h>
#include <mpc-walkgen/function/zebulon_com_centering_objective.h>
#include <mpc-walkgen/function/zebulon_com_constraint.h>
#include <mpc-walkgen/qpsolverfactory.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/noncopyable.hpp>

#ifdef _MSC_VER
# pragma warning( push )
// C4251: class needs to have DLL interface
// C4275: non dll-interface class used as base for dll-interface class
# pragma warning( disable: 4251 4275)
#endif

namespace MPCWalkgen
{
  template <typename Scalar>
  class MPC_WALKGEN_API HumanoidWalkgen : boost::noncopyable
  {
    TEMPLATE_TYPEDEF(Scalar)
  public:
    HumanoidWalkgen();
    ~HumanoidWalkgen();

    /// \brief Set the number of QP samples within the preview window
    void setNbSamples(int nbSamples);
    /// \brief Set the QP sampling period in seconds
    void setSamplingPeriod(Scalar samplingPeriod);
    /// \brief Set the step period in seconds. It must be a multiple of
    ///  the QP sampling period
    void setStepPeriod(Scalar stepPeriod);

    /// \brief Set the duration of the initial double support
    void setInitialDoubleSupportLength(Scalar initialDoubleSupportLength);

    ///  \brief Set the convex polygon of the positions that left and right foot can reach
    ///   according to their kinematic constraints (in local frame)
    void setLeftFootKinematicConvexPolygon(const ConvexPolygon<Scalar>& convexPolygon);
    void setRightFootKinematicConvexPolygon(const ConvexPolygon<Scalar>& convexPolygon);

    ///  \brief Set the simple support CoP convex polygon for left and right foot
    ///         in local frame
    void setLeftFootCopConvexPolygon(const ConvexPolygon<Scalar>& convexPolygon);
    void setRightFootCopConvexPolygon(const ConvexPolygon<Scalar>& convexPolygon);

    /// \brief Set the velocity reference in world frame
    void setVelRefInWorldFrame(const VectorX& velRef);
    /// \brief Set the angular velocity reference around Z axis in world frame
    void setAngularVelRefInWorldFrame(const VectorX& angularVelRef);


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

    inline const VectorX& getLeftFootStateX() const
    {return feetSupervisor_.getLeftFootStateX();}

    inline const VectorX& getLeftFootStateY() const
    {return feetSupervisor_.getLeftFootStateY();}

    inline const VectorX& getLeftFootStateZ() const
    {return feetSupervisor_.getLeftFootStateZ();}

    inline const VectorX& getRightFootStateX() const
    {return feetSupervisor_.getRightFootStateX();}

    inline const VectorX& getRightFootStateY() const
    {return feetSupervisor_.getRightFootStateY();}

    inline const VectorX& getRightFootStateZ() const
    {return feetSupervisor_.getRightFootStateZ();}

    inline const VectorX& getComStateX() const
    {return lipModel_.getStateX();}

    inline const VectorX& getComStateY() const
    {return lipModel_.getStateY();}

    inline const VectorX& getComStateZ() const
    {return lipModel_.getStateZ();}

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
    void setWeightings(const HumanoidWalkgenWeighting<Scalar>& weighting);
    void setConfig(const HumanoidWalkgenConfig<Scalar>& config);
    /// \brief Enable robot walk
    void setMove(bool move);

    //TODO: more doc
    /// \brief solve the given QP problem of the form:
    ///        1/2*xT.H.x + xT.g
    ///        under the following constraints:
    ///        bl <= A.x <= bu
    ///        xl <= x <= xu
    /// \param feedBackPeriod: in seconds, the time between each call
    ///        of the MPC. This is also the period waited for before
    ///        new samples are sent to the actuators
    bool solve(Scalar feedBackPeriod);

  private:
    void computeConstantPart();
    void convertCopInLFtoComJerk();

  private:

    HumanoidFeetSupervisor<Scalar> feetSupervisor_;
    LIPModel<Scalar> lipModel_;

    HumanoidLipComVelocityTrackingObjective<Scalar> velTrackingObj_;
    HumanoidLipComJerkMinimizationObjective<Scalar> jerkMinObj_;
    HumanoidCopCenteringObjective<Scalar> copCenteringObj_;

    HumanoidCopConstraint<Scalar> copConstraint_;
    HumanoidFootConstraint<Scalar> footConstraint_;

    /// \brief A vector containing all possible sizes of QPsolver
    boost::ptr_vector< QPSolver<Scalar> > qpoasesSolverVec_;

    VectorX dX_;
    /// \brief Solution of the QP problem: CoP position in local frame and
    ///        previewed footsteps positions in world frame.
    VectorX X_;
    /// \brief Transformed solution of the QP problem: CoM jerk in world frame and
    ///        previewed foosteps positions in world frame.
    VectorX transformedX_; //TODO: Change this ugly name


    /// \brief QPMatrices is a struct containing the matrices of the QP problem:
    ///        1/2*xT.H.x + xT.g
    ///        under the following constraints:
    ///        bl <= A.x <= bu
    ///        xl <= x <= xu
    ///        Here we use a vector containing all possible sizes of QPMatrices
    std::vector< QPMatrices<Scalar> > qpMatrixVec_;

    HumanoidWalkgenWeighting<Scalar> weighting_;
    HumanoidWalkgenConfig<Scalar> config_;

    int maximumNbOfConstraints_;
    int maximumNbOfSteps_;

    bool move_;
    bool firstCallSinceLastDS_;

  };
}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
