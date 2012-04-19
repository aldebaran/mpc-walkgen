#include <mpc-walkgen/mpc-debug.h>
#include <mpc-walkgen/gettimeofday.h>
#ifdef WIN32
# include <Windows.h>
#endif // WIN32

using namespace MPCWalkgen;

MPCDebug::MPCDebug(bool enable)
	:enable_(enable)
{}

MPCDebug::~MPCDebug(){}

void MPCDebug::getTime(int id, bool start){
	if (enable_){
		struct timeval t;
		gettimeofday(&t,0);
		double time=t.tv_sec + 0.000001*t.tv_usec;
		if (nbCount_.count(id)==0){
			nbCount_[id]=0;
			startTime_[id]=0;
			endTime_[id]=0;
		}
		if (start){
			startTime_[id]=startTime_[id]+time;
		}else{
			endTime_[id]=endTime_[id]+time;
			nbCount_[id]=nbCount_[id]+1;
		}
	}
}


double MPCDebug::computeInterval(int id, TimeUnit unit){
	if (enable_){
		switch(unit){
			case us:
				return 1000000*(endTime_[id]-startTime_[id])/nbCount_[id];
			break;
			case ms:
				return 1000*(endTime_[id]-startTime_[id])/nbCount_[id];
			break;
			default:
				return (endTime_[id]-startTime_[id])/nbCount_[id];
			break;
		}
	}else{
		return 0;
	}
}

int MPCDebug::nbIntervals(int id){
	if (enable_){
		if (nbCount_.count(id)==0){
			nbCount_[id]=0;
			startTime_[id]=0;
			endTime_[id]=0;
		}
		return nbCount_[id];
	}else{
		return 0;
	}
}

void MPCDebug::reset(int id){
	if (enable_){
		startTime_[id]=0;
		endTime_[id]=0;
		nbCount_[id]=0;
	}
}

void MPCDebug::reset(){
	if (enable_){
		std::map<int,double> ::iterator it;
		std::map<int,int> ::iterator it2;
		for(it=startTime_.begin();it!=startTime_.end();++it){
			(*it).second=0;
		}
		for(it=endTime_.begin();it!=endTime_.end();++it){
			(*it).second=0;
		}
		for(it2=nbCount_.begin();it2!=nbCount_.end();++it2){
			(*it2).second=0;
		}
	}
}
