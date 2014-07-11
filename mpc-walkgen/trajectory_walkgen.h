////////////////////////////////////////////////////////////////////////////////
///
///\file trajectory_walkgen.h
///\brief Main program for controlling over a trajectory
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_TRAJECTORY_WALKGEN_H
#define MPC_WALKGEN_TRAJECTORY_WALKGEN_H

#include <mpc-walkgen/api.h>
#include <mpc-walkgen/type.h>


#include <mpc-walkgen/model/no_dynamic_model.h>
#include <mpc-walkgen/function/trajectory_jerk_minimization_objective.h>
#include <mpc-walkgen/function/trajectory_position_tracking_objective.h>
#include <mpc-walkgen/function/trajectory_velocity_tracking_objective.h>
#include <mpc-walkgen/function/trajectory_motion_constraint.h>


#include <mpc-walkgen/qpsolverfactory.h>
#include <boost/scoped_ptr.hpp>

#include <mpc-walkgen/trajectory_walkgen_type.h>
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
  class MPC_WALKGEN_API TrajectoryWalkgen : boost::noncopyable
  {
    TEMPLATE_TYPEDEF(Scalar)
  public:
    TrajectoryWalkgen();
    ~TrajectoryWalkgen();

    void setNbSamples(int nbSamples);
    void setSamplingPeriod(Scalar samplingPeriod);

    void setVelRefInWorldFrame(const VectorX& velRef);
    void setPosRefInWorldFrame(const VectorX& posRef);

    void setVelLimit(Scalar limit);
    void setAccLimit(Scalar limit);
    void setJerkLimit(Scalar limit);

    void setState(const VectorX& state);

    void setWeightings(const TrajectoryWalkgenWeighting<Scalar>& weighting);
    void setConfig(const TrajectoryWalkgenConfig<Scalar>& config);

    bool solve(Scalar feedBackPeriod);

    const VectorX& getState() const;
    const Scalar getJerk() const;

  private:
    void computeConstantPart();

  private:
    boost::scoped_ptr< QPSolver<Scalar> > qpoasesSolver_;

    NoDynamicModel<Scalar> noDynModel_;

    TrajectoryJerkMinimizationObjective<Scalar> jerkMinObj_;
    VelocityTrackingObjective<Scalar> velTrackingObj_;
    PositionTrackingObjective<Scalar> posTrackingObj_;
    MotionConstraint<Scalar> motionConstraint_;

    TrajectoryWalkgenWeighting<Scalar> weighting_;
    TrajectoryWalkgenConfig<Scalar> config_;

    VectorX dX_;
    VectorX X_;

    QPMatrices<Scalar> qpMatrix_;
  };

}
#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
