/*
 */


#include <mpc-walkgen/zebulon/walkgen-abstract.h>

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

void makeScilabFile(std::string type, double time);
void dumpTrajectory(MPCSolution & result, std::vector<std::ofstream*> & data_vec);
bool checkFiles(std::ifstream & f1, std::ifstream & f2);
int copyFile(const std::string & source, const std::string & destination);

int main() {
	std::cout << "0%" << std::endl;
	// Logging:
	// --------
	const int nbFile=7;

	std::vector<std::string> name_vec(nbFile);
	name_vec[0]="CoM_X";
	name_vec[1]="CoM_Y";
	name_vec[2]="CoM_Yaw";
	name_vec[3]="CoP_X";
	name_vec[4]="CoP_Y";
	name_vec[5]="BAS_X";
	name_vec[6]="BAS_Y";

	std::vector<std::ofstream*> data_vec(nbFile);
	std::vector<std::ifstream*> ref_vec(nbFile);
	for (int i = 0; i < nbFile; ++i) {
		data_vec[i] = new std::ofstream((name_vec[i]+".data").c_str());
		ref_vec[i] = new std::ifstream((name_vec[i]+".ref").c_str());
	}



	// Create and initialize generator:
	// -------------------------------

	WalkgenAbstract * walk = createWalkgen(CURRENT_QPSOLVERTYPE);

	MPCData mpcData;
	mpcData.ponderation.basePosition[0]=0;
	mpcData.ponderation.OrientationPosition[0]=0;
	walk->init(mpcData);

	// Run:
	// ----
	std::cout << "10%" << std::endl;
	double velocity = 0.4;
	walk->velReferenceInLocalFrame(0, 0, 0);
	walk->online(0.0);
	double t = 0;
	for (; t < 5; t += 0.005){
		MPCSolution result = walk->online(t);
		dumpTrajectory(result, data_vec);
	}
	std::cout << "30%" << std::endl;
	walk->velReferenceInLocalFrame(0, 0, 0);
	for (; t < 8; t += 0.005){
		MPCSolution result = walk->online(t);
		dumpTrajectory(result, data_vec);
	}
	std::cout << "55%" << std::endl;
	walk->velReferenceInLocalFrame(0, velocity, 0);
	for (; t < 12; t += 0.005){
		MPCSolution result = walk->online(t);
		dumpTrajectory(result, data_vec);
	}
	BodyState baseState = walk->bodyState(BASE);
	baseState.x(2) -= 1;
	walk->bodyState(BASE, baseState);

	for (; t < 20; t += 0.005){
		MPCSolution result = walk->online(t);
		dumpTrajectory(result, data_vec);
	}
	std::cout << "80%" << std::endl;
	walk->velReferenceInLocalFrame(velocity, 0, 0);
	for (; t < 20; t += 0.005){
		MPCSolution result = walk->online(t);
		dumpTrajectory(result, data_vec);
	}
	for(unsigned i = 0; i < data_vec.size(); ++i){
		data_vec[i]->close();
	}


	// Reopen the files:
	// -----------------
	std::vector<std::ifstream*> check_vec(nbFile);
	for(int i = 0;i < nbFile; ++i){
		check_vec[i] = new std::ifstream((name_vec[i]+".data").c_str());
	}


	bool success = true;
	for(unsigned i = 0; i < check_vec.size();++i){
		// if the reference file exists, compare with the previous version.
		if (*ref_vec[i]){
			if (!checkFiles(*check_vec[i],*ref_vec[i])){
				success = false;
			}
		}
		// otherwise, create it
		else{
			copyFile((name_vec[i]+".data"), (name_vec[i]+".ref"));
		}
		check_vec[i]->close();
		ref_vec[i]->close();
	}
	makeScilabFile("data", t);
	makeScilabFile("ref", t);

	for (int i=0; i < nbFile; ++i) {
		delete check_vec[i];
		delete ref_vec[i];
		delete data_vec[i];
	}
	delete walk;
	std::cout << "100%" << std::endl;
	return (success)?0:1;
}

void dumpTrajectory(MPCSolution &result, std::vector<std::ofstream*> &data_vec) {
	for (int i = 0; i < result.state_vec[0].CoMTrajX_.rows(); ++i) {
	  *data_vec[0] << result.state_vec[0].CoMTrajX_(i) << " " << result.state_vec[1].CoMTrajX_(i) << " " << result.state_vec[2].CoMTrajX_(i) << std::endl;
	  *data_vec[1] << result.state_vec[0].CoMTrajY_(i) << " " << result.state_vec[1].CoMTrajY_(i) << " " << result.state_vec[2].CoMTrajY_(i) << std::endl;
	  *data_vec[2] << result.state_vec[0].CoMTrajYaw_(i) << " " << result.state_vec[1].CoMTrajYaw_(i) << " " << result.state_vec[2].CoMTrajYaw_(i) << std::endl;
	  *data_vec[3] << result.CoPTrajX(i)  << std::endl;
	  *data_vec[4] << result.CoPTrajY(i) << std::endl;
	  *data_vec[5] << result.state_vec[0].baseTrajX_(i) << " " << result.state_vec[1].baseTrajX_(i) << " " << result.state_vec[2].baseTrajX_(i) << std::endl;
	  *data_vec[6] << result.state_vec[0].baseTrajY_(i) << " " << result.state_vec[1].baseTrajY_(i) << " " << result.state_vec[2].baseTrajY_(i) << std::endl;
	}
}

void makeScilabFile(std::string type, double time) {
	std::ofstream sci(("plot"+type+".sci").c_str());

	sci.close();
}

bool checkFiles(std::ifstream & fich1, std::ifstream & fich2) {
	bool equal=1;
	if (fich1 && fich2) {
		std::string lignef1;
		std::string lignef2;
		while (std::getline( fich1, lignef1) && std::getline( fich2, lignef2) && equal) {
			if (strcmp(lignef1.c_str(),lignef2.c_str())!=0) {
				equal = 0;
			}
		}
	}

	return equal;
}

int copyFile(const std::string & source, const std::string & destination) {
	std::ifstream ifs(source.c_str(), std::ios::binary);
	std::ofstream ofs(destination.c_str(), std::ios::binary);
	ofs << ifs.rdbuf();
	ofs.close();
	ifs.close();

	return 0;
}
