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
	supportState_vec.resize(0);
	supportOrientation_vec.resize(0);
	  supportTrunkOrientation_vec.resize(0);
  }

  MPCData::MPCData()
  :QPSamplingPeriod(0.1)
  ,MPCSamplingPeriod(0.005)
  ,simSamplingPeriod(0.005)
  ,QPNbSamplings(16)
  ,stepPeriod(0.8)
  ,DSPeriod(1e9)
  ,DSSSPeriod(0.8)
  ,nbStepSSDS(2) {
  }

  int MPCData::iterationNumberFeedback(double firstIterationduration) const{
	  return static_cast<int> (round(firstIterationduration / MPCSamplingPeriod)-1 );
  }

  int MPCData::nbIterationFeedback() const{
	  return static_cast<int> (round(QPSamplingPeriod / MPCSamplingPeriod) );
  }

  int MPCData::nbIterationSimulation() const{
	  return static_cast<int> (round(MPCSamplingPeriod / simSamplingPeriod) );
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

  // TODO: Necessary?
  ConvexHull & ConvexHull::operator=(const ConvexHull & hull){
  	x=hull.x;
  	y=hull.y;
  	A=hull.A;
  	B=hull.B;
  	C=hull.C;
  	D=hull.D;

  	return *this;
  }

  void ConvexHull::resize(int size){
  	if (size != x.rows()){
  		x.setZero(size);
  		y.setZero(size);
  		A.setZero(size);
  		B.setZero(size);
  		C.setZero(size);
  		D.setZero(size);
  	}else{

  		x.fill(0);
  		y.fill(0);
  		A.fill(0);
  		B.fill(0);
  		C.fill(0);
  		D.fill(0);
  	}
  }

  void ConvexHull::rotate(double yaw) {
  	if (yaw == 0)
  		return;

  	double xOld, yOld;
  	int size = x.rows();
  	for(int i = 0; i < size; ++i){
  		xOld = x(i);
  		yOld = y(i);
  		x(i) = (xOld*std::cos(yaw) - yOld*std::sin(yaw));
  		y(i) = (xOld*std::sin(yaw) + yOld*std::cos(yaw));
  	}
  }

  void ConvexHull::computeLinearSystem(const Foot & foot) {
  	double dx,dy,dc,x1,y1,x2,y2;
  	unsigned nbRows = x.rows();

  	double sign;
  	if(foot == LEFT){
  		sign = 1.0;
  	}else{
  		sign = -1.0;
  	}
  	for( unsigned i=0; i<nbRows;++i ){
  		y1 = y(i);
  		y2 = y((i+1)%nbRows);
  		x1 = x(i);
  		x2 = x((i+1)%nbRows);

  		dx = sign*(y1-y2);
  		dy = sign*(x2-x1);
  		dc = dx*x1+dy*y1;


  		A(i) = dx;
  		B(i) = dy;
  		D(i) = dc;
  	}
  }


}
