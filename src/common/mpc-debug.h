#ifndef MPC_DEBUG
#define MPC_DEBUG

////////////////////////////////////////////////////////////////////////////////
///
///\file	mpc-debug.h
///\brief	A class to debug the program
///\author	Lafaye Jory
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////

#include <map>

namespace MPCWalkgen{

	enum TimeUnit{
		us,
		ms,
		s
	};

	class MPCDebug{
		public:
			MPCDebug(bool enable);
			~MPCDebug();

			void getTime(int id, bool start);
			double computeInterval(int id, TimeUnit unit = us);
			int nbIntervals(int id);

			void reset(int id);
			void reset();

		private:
			std::map<int,double> startTime_;
			std::map<int,double> endTime_;
			std::map<int,int> nbCount_;

			bool enable_;

	};

}



#endif //MPC_DEBUG
