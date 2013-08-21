#include <mpc-walkgen/shared_type.h>

using namespace MPCWalkgen;

Weighting::Weighting()
:velocityTracking(0.0)
,positionTracking(0.0)
,copCentering(0.0)
,jerkMinimization(0.0)
,tiltMinimization(0.0)
{}

Config::Config()
:withCopConstraints(true)
,withComConstraints(true)
,withBaseMotionConstraints(true)
{}
