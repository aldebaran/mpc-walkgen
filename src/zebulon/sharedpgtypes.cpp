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

int MPCData::nbFeedbackSamplesLeft(double firstIterationduration) const{
  return static_cast<int> (round(firstIterationduration / MPCSamplingPeriod)-1 );
}

int MPCData::nbFeedbackSamplesStandard() const{
  return static_cast<int> (round(QPSamplingPeriod / MPCSamplingPeriod) );
}

int MPCData::nbSamplesControl() const{
  return static_cast<int> (round(MPCSamplingPeriod / actuationSamplingPeriod) );
}


RobotData::RobotData()
  :CoMHeight(0.45)
  ,b(0.31-0.06)
  ,h(0.26-0.06)
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
  CopCentering[0]        = 0.01;
  CoMCentering[0]        = 0.001;
  CoMJerkMin[0]          = 0.00001;
  baseJerkMin[0]         = 0.000001;
  baseInstantVelocity[0] = 1;
  basePosition[0]        = 1;

  OrientationInstantVelocity[0] = 1;
  OrientationPosition[0]        = 1;
  OrientationJerkMin[0]         = 0.00001;

  activePonderation = 0;
}


