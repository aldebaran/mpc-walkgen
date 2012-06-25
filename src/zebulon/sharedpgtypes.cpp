#include <mpc-walkgen/zebulon/sharedpgtypes.h>
#include "../common/tools.h"

using namespace MPCWalkgen::Zebulon;

MPCSolution::MPCSolution()
:state_vec(3)
{}

MPCData::MPCData()
:QPSamplingPeriod(0.16)
,MPCSamplingPeriod(0.02)
,actuationSamplingPeriod(0.02)
,nbSamplesQP(10)
{}

int MPCData::nbSamplesControl() const{
  return static_cast<int> (round(MPCSamplingPeriod / actuationSamplingPeriod) );
}


RobotData::RobotData()
  :CoMHeight(0.45)
  ,b(0.31-0.02)
  ,h(0.26-0.02)
  ,baseLimit(3)
  ,orientationLimit(3)
{
  baseLimit[0]=0.83;
  baseLimit[1]=1;
  baseLimit[2]=2;
  orientationLimit[0]=2*3.14;
  orientationLimit[1]=3.14;
  orientationLimit[2]=3.14;
  comLimitX=0.12*0.8;
  comLimitY=0.035*0.8;
}

QPPonderation::QPPonderation(int nb)
  :baseInstantVelocity(nb)
  ,basePosition(nb)
  ,CopCentering(nb)
  ,CoMCentering(nb)
  ,CoMJerkMin(nb)
  ,baseJerkMin(nb)
  ,OrientationInstantVelocity(nb)
  ,OrientationPosition(nb)
  ,OrientationJerkMin(nb)
{
  CopCentering[0]        = 0.1;
  CoMCentering[0]        = 0.01;
  CoMJerkMin[0]          = 0.0001;
  baseJerkMin[0]         = 0.00001;
  baseInstantVelocity[0] = 1;
  basePosition[0]        = 10;

  OrientationInstantVelocity[0] = 1;
  OrientationPosition[0]        = 10;
  OrientationJerkMin[0]         = 0.0001;

  activePonderation = 0;
}


