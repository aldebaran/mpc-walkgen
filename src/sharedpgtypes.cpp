/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010, 
 *
 * Andrei Herdt
 * Olivier Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the 
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
#include <mpc-walkgen/sharedpgtypes.h>

#include <iostream>
#include <fstream>

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
  ,MPCSamplingPeriod(0.1)
  ,actuationSamplingPeriod(0.005)
  ,nbSamplesQP(16)
  ,stepPeriod(0.8)
  ,DSPeriod(1e9)
  ,DSSSPeriod(0.8)
  ,nbStepSSDS(2) {
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

  int MPCData::nbFeedbackSamplesLeft(double firstIterationduration) const{
	  return static_cast<int> (round(firstIterationduration / MPCSamplingPeriod)-1 );
  }

  int MPCData::nbFeedbackSamplesStandard() const{
	  return static_cast<int> (round(QPSamplingPeriod / MPCSamplingPeriod) );
  }

  int MPCData::nbIterationSimulation() const{
	  return static_cast<int> (round(MPCSamplingPeriod / actuationSamplingPeriod) );
  }

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

  BodyState::BodyState(){
	  reset();
  }

  void BodyState::reset(){
	  x.fill(0);
	  y.fill(0);
	  z.fill(0);
	  yaw.fill(0);
	  pitch.fill(0);
	  roll.fill(0);
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
