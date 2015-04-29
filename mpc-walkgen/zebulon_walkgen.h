////////////////////////////////////////////////////////////////////////////////
///
///\file zebulon_walkgen.h
///\brief Main program for Zebulon
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_ZEBULON_WALKGEN_H
#define MPC_WALKGEN_ZEBULON_WALKGEN_H

#include <mpc-walkgen/api.h>
#include <mpc-walkgen/type.h>
#include <mpc-walkgen/model/lip_model.h>
#include <mpc-walkgen/model/zebulon_base_model.h>
#include <mpc-walkgen/function/zebulon_base_velocity_tracking_objective.h>
#include <mpc-walkgen/function/zebulon_base_position_tracking_objective.h>
#include <mpc-walkgen/function/zebulon_jerk_minimization_objective.h>
#include <mpc-walkgen/function/zebulon_tilt_minimization_objective.h>
#include <mpc-walkgen/function/zebulon_tilt_velocity_minimization_objective.h>
#include <mpc-walkgen/function/zebulon_cop_centering_objective.h>
#include <mpc-walkgen/function/zebulon_com_centering_objective.h>
#include <mpc-walkgen/function/zebulon_cop_constraint.h>
#include <mpc-walkgen/function/zebulon_com_constraint.h>
#include <mpc-walkgen/function/zebulon_base_motion_constraint.h>

#include <mpc-walkgen/qpsolverfactory.h>
#include <boost/scoped_ptr.hpp>

#include <mpc-walkgen/function/zebulon_tilt_motion_constraint.h>
#include <mpc-walkgen/zebulon_walkgen_type.h>
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
  class MPC_WALKGEN_API ZebulonWalkgen : boost::noncopyable
  {
    TEMPLATE_TYPEDEF(Scalar)
  public:
    ZebulonWalkgen();
    ~ZebulonWalkgen();

    void setNbSamples(int nbSamples);
    void setSamplingPeriod(Scalar samplingPeriod);

    void setGravity(const Vector3& gravity);
    void setBaseCopConvexPolygon(const ConvexPolygon<Scalar>& convexPolygon);
    void setBaseComConvexPolygon(const ConvexPolygon<Scalar>& convexPolygon);
    // legacy
    void setBaseCopHull(const vectorOfVector3& p);
    void setBaseComHull(const vectorOfVector3& p);

    void setComBodyHeight(Scalar comHeight);
    void setComBaseHeight(Scalar comHeight);
    void setBodyMass(Scalar mass);
    void setBaseMass(Scalar mass);

    void setVelRefInWorldFrame(const VectorX& velRef);
    void setPosRefInWorldFrame(const VectorX& posRef);
    void setCopRefInLocalFrame(const VectorX& copRef);
    void setComRefInLocalFrame(const VectorX& comRef);
    void setComAndCopRefInWorldFrame(const VectorX& comPosRefInWorldFrame,
                                     const VectorX& comAccRefInWorldFrame,
                                     const VectorX& basePosRefInWorldFrame,
                                     const VectorX& baseAccRefInWorldFrame);

    void setBaseVelLimit(Scalar limit);
    void setBaseAccLimit(Scalar limit);
    void setBaseJerkLimit(Scalar limit);

    void setBaseStateX(const VectorX& state);
    void setBaseStateY(const VectorX& state);
    void setBaseStateRoll(const VectorX& state);
    void setBaseStatePitch(const VectorX& state);
    void setBaseStateYaw(const VectorX& state);
    void setComStateX(const VectorX& state);
    void setComStateY(const VectorX& state);

    void setTiltContactPointOnTheGroundInLocalFrameX(Scalar pos);
    void setTiltContactPointOnTheGroundInLocalFrameY(Scalar pos);

    void setWeightings(const ZebulonWalkgenWeighting<Scalar>& weighting);
    void setConfig(const ZebulonWalkgenConfig<Scalar>& config);

    bool solve(Scalar feedBackPeriod);

    const VectorX& getBaseStateX() const;
    const VectorX& getBaseStateY() const;
    const VectorX& getComStateX() const;
    const VectorX& getComStateY() const;

  private:
    void computeConstantPart();

    void computeNormalizationFactor(MatrixX& Q, MatrixX& A);

  private:
    LIPModel<Scalar> lipModel_;
    BaseModel<Scalar> baseModel_;

    BaseVelocityTrackingObjective<Scalar> velTrackingObj_;
    BasePositionTrackingObjective<Scalar> posTrackingObj_;
    JerkMinimizationObjective<Scalar> jerkMinObj_;
    TiltMinimizationObjective<Scalar> tiltMinObj_;
    TiltVelMinimizationObjective<Scalar> tiltVelMinObj_;
    CopCenteringObjective<Scalar> copCenteringObj_;
    ComCenteringObjective<Scalar> comCenteringObj_;
    CopConstraint<Scalar> copConstraint_;
    ComConstraint<Scalar> comConstraint_;
    BaseMotionConstraint<Scalar> baseMotionConstraint_;
    TiltMotionConstraint<Scalar> tiltMotionConstraint_;

    boost::scoped_ptr< QPSolver<Scalar> > qpoasesSolver_;

    ZebulonWalkgenWeighting<Scalar> weighting_;
    ZebulonWalkgenConfig<Scalar> config_;

    VectorX dX_;
    VectorX X_;
    VectorX B_;

    QPMatrices<Scalar> qpMatrix_;

    Scalar invObjNormFactor_;
    Scalar invCtrNormFactor_;
  };

}
#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
