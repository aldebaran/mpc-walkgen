#include <mpc-walkgen/sharedpgtypes-humanoid.h>



namespace MPCWalkgen
{

MPCSolution::MPCSolution()
:useWarmStart(true)
,state_vec(3)
{}

void MPCSolution::reset(){
	supportStates_vec.resize(0);
	supportOrientations_vec.resize(0);
	supportTrunkOrientations_vec.resize(0);
}

  MPCData::MPCData()
  :QPSamplingPeriod(0.1)
  ,MPCSamplingPeriod(0.005)
  ,actuationSamplingPeriod(0.005)
  ,nbSamplesQP(16)
  ,stepPeriod(0.8)
  ,DSPeriod(1e9)
  ,DSSSPeriod(0.8)
  ,nbStepSSDS(2)
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



  RobotData::RobotData(const FootData &leftFoot, const FootData &rightFoot,
        const HipYawData &leftHipYaw, const HipYawData &rightHipYaw,
        double mass)
  :	CoMHeight(0.814)
        ,freeFlyingFootMaxHeight(0.05)
        ,leftFoot(leftFoot)
        ,rightFoot(rightFoot)
        ,leftHipYaw(leftHipYaw)
        ,rightHipYaw(rightHipYaw)
        ,robotMass(mass)
        ,leftFootPos()
        ,rightFootPos()
        ,leftFootHull()
        ,rightFootHull()
        ,CoPLeftSSHull()
        ,CoPRightSSHull()
        ,CoPLeftDSHull()
        ,CoPRightDSHull() {
          leftFootPos << 0.00949035, 0.095, 0;
          rightFootPos << 0.00949035, -0.095, 0;
  }
  RobotData::RobotData(){}

  QPPonderation::QPPonderation(int nb)
  :instantVelocity(nb)
  ,CopCentering(nb)
  ,JerkMin(nb) {
          CopCentering[0]    = 0.0001;
          JerkMin[0]         = 0.001;
          instantVelocity[0] = 1;

	  CopCentering[1]    = 10;
	  JerkMin[1]         = 0.001;
	  instantVelocity[1] = 1;

          activePonderation = 1;
  }


  HipYawData::HipYawData()
  : lowerBound(-0.523599)
  , upperBound(0.785398)
  , lowerVelocityBound(-3.54108)
  , upperVelocityBound(3.54108)
  , lowerAccelerationBound(-0.1)
  , upperAccelerationBound(0.1) {
  }
}
