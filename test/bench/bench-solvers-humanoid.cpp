
/*
 */



#include <mpc-walkgen/humanoid/sharedpgtypes.h>
#include <mpc-walkgen/humanoid/walkgen-abstract.h>
#include <../src/common/mpc-debug.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <stdio.h>

using namespace Eigen;
using namespace MPCWalkgen;
using namespace Humanoid;
int main() {

	std::cout << "0%" << std::endl;
	// Robot parameters (HRP-2):
	// -------------------------
	FootData leftFoot;
	leftFoot.anklePositionInLocalFrame << 0, 0, 0.105;
	leftFoot.soleHeight = 0.138;
	leftFoot.soleWidth = 0.2172;
	FootData rightFoot;
	rightFoot.anklePositionInLocalFrame << 0, 0, 0.105;
	rightFoot.soleHeight = 0.138;
	rightFoot.soleWidth = 0.2172;

	HipYawData leftHipYaw;
	leftHipYaw.lowerBound = -0.523599;
	leftHipYaw.upperBound = 0.785398;
	leftHipYaw.lowerVelocityBound = -3.54108;
	leftHipYaw.upperVelocityBound = 3.54108;
	leftHipYaw.lowerAccelerationBound = -0.1;
	leftHipYaw.upperAccelerationBound = 0.1;
	HipYawData rightHipYaw = leftHipYaw;

	MPCData mpcData;
	RobotData robotData(leftFoot, rightFoot, leftHipYaw, rightHipYaw, 0.0);

	// Feasible hulls:
	// ---------------
	const int nbVertFeet = 5;
	// Feasible foot positions
	double DefaultFPosEdgesX[nbVertFeet] = {-0.28, -0.2, 0.0, 0.2, 0.28};
	double DefaultFPosEdgesY[nbVertFeet] = {-0.2, -0.3, -0.4, -0.3, -0.2};

	robotData.leftFootHull.resize(nbVertFeet);
	robotData.rightFootHull.resize(nbVertFeet);
	for (int i=0; i < nbVertFeet; ++i) {
		robotData.leftFootHull.x(i)=DefaultFPosEdgesX[i];
		robotData.leftFootHull.y(i)=DefaultFPosEdgesY[i];
		robotData.rightFootHull.x(i)=DefaultFPosEdgesX[i];
		robotData.rightFootHull.y(i)=-DefaultFPosEdgesY[i];
	}

	// Constraints on the CoP
	const int nbVertCoP = 4;
	double DefaultCoPSSEdgesX[nbVertCoP] = {0.0686, 0.0686, -0.0686, -0.0686};
	double DefaultCoPSSEdgesY[nbVertCoP] = {0.029, -0.029, -0.029, 0.029};
	double DefaultCoPDSEdgesX[nbVertCoP] = {0.0686, 0.0686, -0.0686, -0.0686};
	double DefaultCoPDSEdgesY[nbVertCoP] = {0.029, -0.229, -0.229, 0.029};

	robotData.CoPLeftSSHull.resize(nbVertCoP);
	robotData.CoPRightSSHull.resize(nbVertCoP);
	robotData.CoPLeftDSHull.resize(nbVertCoP);
	robotData.CoPRightDSHull.resize(nbVertCoP);
	for (int i = 0; i < nbVertCoP; ++i) {
		robotData.CoPLeftSSHull.x(i) = DefaultCoPSSEdgesX[i];
		robotData.CoPLeftSSHull.y(i) = DefaultCoPSSEdgesY[i];
		robotData.CoPLeftDSHull.x(i) = DefaultCoPDSEdgesX[i];
		robotData.CoPLeftDSHull.y(i) = DefaultCoPDSEdgesY[i];

		robotData.CoPRightSSHull.x(i) = DefaultCoPSSEdgesX[i];
		robotData.CoPRightSSHull.y(i) =- DefaultCoPSSEdgesY[i];
		robotData.CoPRightDSHull.x(i) = DefaultCoPDSEdgesX[i];
		robotData.CoPRightDSHull.y(i) =- DefaultCoPDSEdgesY[i];
	}

	// Creat and initialize generator:
	// -------------------------------

	WalkgenAbstract * walk1 = createWalkgen(QPSOLVERTYPE_LSSOL);
	WalkgenAbstract * walk2 = createWalkgen(QPSOLVERTYPE_QPOASES);
	walk2->init(robotData, mpcData);
	walk1->init(robotData, mpcData);


	MPCDebug debug(true);

	// Run:
	// ----
	double velocity = 0.25;
	walk2->reference(0, 0, 0);
	walk2->online(0.0);
	walk1->reference(0, 0, 0);
	walk1->online(0.0);
	double t = 0;
	for (; t < 5; t += 0.005){
		debug.getTime(1,true);
		walk2->online(t);
		debug.getTime(1,false);
		debug.getTime(2,true);
		walk1->online(t);
		debug.getTime(2,false);
	}
	std::cout << "25%" << std::endl;
	walk2->reference(velocity, 0, 0);
	walk1->reference(velocity, 0, 0);
	for (; t < 10; t += 0.005){
		debug.getTime(1,true);
		walk2->online(t);
		debug.getTime(1,false);
		debug.getTime(2,true);
		walk1->online(t);
		debug.getTime(2,false);
	}
	std::cout << "50%" << std::endl;
	walk2->reference(0, velocity, 0);
	walk1->reference(0, velocity, 0);
	for (; t < 20; t += 0.005){
		debug.getTime(1,true);
		walk2->online(t);
		debug.getTime(1,false);
		debug.getTime(2,true);
		walk1->online(t);
		debug.getTime(2,false);
	}
	std::cout << "75%" << std::endl;
	walk2->reference(velocity, 0, velocity);
	walk1->reference(velocity, 0, velocity);
	for (; t < 30; t += 0.005){
		debug.getTime(1,true);
		walk2->online(t);
		debug.getTime(1,false);
		debug.getTime(2,true);
		walk1->online(t);
		debug.getTime(2,false);
	}

	std::cout << "100%" << std::endl;

	std::cout << "Walkgen test :" << std::endl;
	std::cout << "Mean iteration duration with QPOASES   : " << debug.computeInterval(1) << " us" << std::endl;
	std::cout << "Mean iteration duration with LSSOL : " << debug.computeInterval(2) << " us" << std::endl;

	return 0;
}
