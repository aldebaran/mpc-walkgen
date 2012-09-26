#include <mpc-walkgen/zebulon/sharedpgtypes.h>
#include "../common/tools.h"

using namespace MPCWalkgen::Zebulon;

MPCSolution::MPCSolution()
:state_vec(5)
{
  CoPTrajX.setZero(1);
  CoPTrajY.setZero(1);
}

MPCSolution::State::State(){
  CoMTrajX_.setZero(1);
  CoMTrajY_.setZero(1);
  CoMTrajYaw_.setZero(1);
  baseTrajX_.setZero(1);
  baseTrajY_.setZero(1);
}

MPCData::MPCData()
:QPSamplingPeriod(0.16)
,MPCSamplingPeriod(0.02)
,actuationSamplingPeriod(0.02)
,nbSamplesQP(10)
{}

int MPCData::nbSamplesControl() const{
  return static_cast<int> (round(MPCSamplingPeriod / actuationSamplingPeriod) );
}

int MPCData::nbSamplesMPC() const{
  return static_cast<int> (round(QPSamplingPeriod / MPCSamplingPeriod) );
}


RobotData::RobotData()
  :CoMHeight(0.45)
  ,copLimitX(0.264*0.8)
  ,copLimitY(0.252*0.8)
  ,deltaComXLocal(0.02)
  ,baseLimit(3)
  ,orientationLimit(3)
  ,comLimitX(0.071*0.8)
  ,comLimitY(0.044*0.8)
  ,gravity(0,0,9.81)
{
  baseLimit[0]=0.83*10;
  baseLimit[1]=1*10;
  baseLimit[2]=50*10;
  orientationLimit[0]=2*3.14;
  orientationLimit[1]=3*3.14;
  orientationLimit[2]=6*3.14;
}

QPPonderation::QPPonderation(int nb)
  :baseInstantVelocity(nb)
  ,basePosition(nb)
  ,basePositionInt(nb)
  ,CopCentering(nb)
  ,CoMCentering(nb)
  ,CoMJerkMin(nb)
  ,baseJerkMin(nb)
  ,OrientationInstantVelocity(nb)
  ,OrientationPosition(nb)
  ,OrientationJerkMin(nb)
  ,activePonderation(0)
{

  // Normal moveTo
  CopCentering[0]        = 1;
  CoMCentering[0]        = 0;
  CoMJerkMin[0]          = 0.001;
  baseJerkMin[0]         = 0;
  baseInstantVelocity[0] = 1;
  basePosition[0]        = 1;
  basePositionInt[0]     = 0;

  OrientationInstantVelocity[0] = 1;
  OrientationPosition[0]        = 1;
  OrientationJerkMin[0]         = 0;

  // More stable moveTo
  CopCentering[1]        = 100;
  CoMCentering[1]        = 0;
  CoMJerkMin[1]          = 0.001;
  baseJerkMin[1]         = 0;
  baseInstantVelocity[1] = 1;
  basePosition[1]        = 1;
  basePositionInt[1]     = 0;

  OrientationInstantVelocity[1] = 1;
  OrientationPosition[1]        = 1;
  OrientationJerkMin[1]         = 0;


  // Normal move
  CopCentering[2]        = 1;
  CoMCentering[2]        = 0;
  CoMJerkMin[2]          = 0.001;
  baseJerkMin[2]         = 0;
  baseInstantVelocity[2] = 1;
  basePosition[2]        = 0;
  basePositionInt[2]     = 0;

  OrientationInstantVelocity[2] = 1;
  OrientationPosition[2]        = 0;
  OrientationJerkMin[2]         = 0;


  // More stable move
  CopCentering[3]        = 100;
  CoMCentering[3]        = 0;
  CoMJerkMin[3]          = 0.001;
  baseJerkMin[3]         = 0;
  baseInstantVelocity[3] = 1;
  basePosition[3]        = 0;
  basePositionInt[3]     = 0;

  OrientationInstantVelocity[3] = 1;
  OrientationPosition[3]        = 0;
  OrientationJerkMin[3]         = 0;

}


