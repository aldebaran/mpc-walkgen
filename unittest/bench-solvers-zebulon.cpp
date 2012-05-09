
/*
 */



#include <mpc-walkgen/sharedpgtypes-zebulon.h>
#include <mpc-walkgen/walkgen-abstract-zebulon.h>
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
using namespace Zebulon;
int main() {

	std::cout << "0%" << std::endl;

	MPCData mpcData;
	RobotData robotData;


	// Creat and initialize generator:
	// -------------------------------

	WalkgenAbstractZebulon * walk1 = createWalkgen(QPSOLVERTYPE_LSSOL);
	WalkgenAbstractZebulon * walk2 = createWalkgen(QPSOLVERTYPE_QPOASES);
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
