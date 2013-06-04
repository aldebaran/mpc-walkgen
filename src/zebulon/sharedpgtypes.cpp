#include <mpc-walkgen/zebulon/sharedpgtypes.h>
#include "../common/tools.h"
#include <iostream>

using namespace MPCWalkgen::Zebulon;

int EnvData::nbObstacleMax=10;

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
,QPNbVariables(nbSamplesQP*4)
,QPNbConstraints(nbSamplesQP*9)
,QPOrientationNbVariables(nbSamplesQP)
,QPOrientationNbConstraints(nbSamplesQP*2)
{}

int MPCData::nbSamplesControl() const{
  return static_cast<int> (round(MPCSamplingPeriod / actuationSamplingPeriod) );
}

int MPCData::nbSamplesMPC() const{
  return static_cast<int> (round(QPSamplingPeriod / MPCSamplingPeriod) );
}

RobotData::RobotData()
  :dAngle(0,0)
  ,CoMHeight(0.45)
  ,copLimitX(0.264*0.8)
  ,copLimitY(0.252*0.8)
  ,deltaComXLocal(-0.042)
  ,baseLimit(3)
  ,orientationLimit(3)
  ,comLimitX(0.071*0.8)
  ,comLimitY(0.044*0.5)
  ,gravity(0,0,9.81)
  ,previousGravity(0,0,9.81)
{
  baseLimit[0]=0.83*10;
  baseLimit[1]=1*10;
  baseLimit[2]=50*10;
  orientationLimit[0]=2*3.14;
  orientationLimit[1]=3*3.14;
  orientationLimit[2]=6*3.14;
}

QPWeighting::QPWeighting(int nb)
  :nbPartialWeightings(nb)
  ,baseInstantVelocity(2*(nb+1))
  ,basePosition(2*(nb+1))
  ,basePositionInt(2*(nb+1))
  ,CopCentering(2*(nb+1))
  ,CoMCentering(2*(nb+1))
  ,CoMJerkMin(2*(nb+1))
  ,baseJerkMin(2*(nb+1))
  ,angularMomentumMin(2*(nb+1))
  ,torqueMin(2*(nb+1))
  ,OrientationInstantVelocity(2*(nb+1))
  ,OrientationPosition(2*(nb+1))
  ,OrientationJerkMin(2*(nb+1))
  ,activeWeighting(0)
{
  //moveTo
  CopCentering[0]        = 1;
  CoMCentering[0]        = 0;
  CoMJerkMin[0]          = 0.0001;
  angularMomentumMin[0]  = 0;
  torqueMin[0]           = 0;
  baseJerkMin[0]         = 0;
  baseInstantVelocity[0] = 10;
  basePosition[0]        = 10;
  basePositionInt[0]     = 0;

  OrientationInstantVelocity[0] = 1;
  OrientationPosition[0]        = 1;
  OrientationJerkMin[0]         = 0;


  //move
  CopCentering[nb+1]        = 1;
  CoMCentering[nb+1]        = 0;
  CoMJerkMin[nb+1]          = 0;
  angularMomentumMin[nb+1]  = 0;//2;
  torqueMin[nb+1]           = 0;//0.1;
  baseJerkMin[nb+1]         = 0;
  baseInstantVelocity[nb+1] = 15;
  basePosition[nb+1]        = 0;
  basePositionInt[nb+1]     = 0;

  OrientationInstantVelocity[nb+1] = 1;
  OrientationPosition[nb+1]        = 1;
  OrientationJerkMin[nb+1]         = 0;


  for(int i=0; i<nb; ++i)
  {
    //more stable moveTo
    const double factor = static_cast<double>(i)/static_cast<double>(nb);
    CopCentering[i+1]        = 1;//+factor*factor*100;
    CoMCentering[i+1]        = 0;
    CoMJerkMin[i+1]          = 0.0001;
    angularMomentumMin[i+1]  = 0;
    torqueMin[i+1]           = 1;
    baseJerkMin[i+1]         = 0;
    baseInstantVelocity[i+1] = 10;
    basePosition[i+1]        = 10;
    basePositionInt[i+1]     = 0;

    OrientationInstantVelocity[i+1] = 1;
    OrientationPosition[i+1]        = 1;
    OrientationJerkMin[i+1]         = 0;

    //more stable move
    CopCentering[nb+i+2]        = 1;
    CoMCentering[nb+i+2]        = 0;
    CoMJerkMin[nb+i+2]          = 0;
    angularMomentumMin[nb+i+2]  = 0;//2;
    torqueMin[nb+i+2]           = 0;//0.1+factor*50+factor*factor*50;
    baseJerkMin[nb+i+2]         = 0;
    baseInstantVelocity[nb+i+2] = 15;
    basePosition[nb+i+2]        = 0;
    basePositionInt[nb+i+2]     = 0;

    OrientationInstantVelocity[nb+i+2] = 1;
    OrientationPosition[nb+i+2]        = 1;
    OrientationJerkMin[nb+i+2]         = 0;


  }
}


