#ifndef RIGID_BODY
#define RIGID_BODY

////////////////////////////////////////////////////////////////////////////////
///
///\file	rigid-body.h
///\brief	A class to store rigid bodies
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author	Keith François
///\version	1.0
///\date	16/02/12
///
////////////////////////////////////////////////////////////////////////////////


#include <mpc-walkgen/types.h>
#include <mpc-walkgen/interpolation.h>
#include <vector>

namespace MPCWalkgen{

	class RigidBody{

		public:
			RigidBody(const MPCData * generalData,
					const RobotData * robotData,
					 const Interpolation * interpolation);
			virtual ~RigidBody();

			inline const BodyState & state() const{return state_;}

			inline void state(const BodyState & s){state_=s;}

			const DynamicMatrix & dynamic(DynamicMatrixType type) const;

			void firstSamplingPeriod(double firstSamplingPeriod);

			void computeDynamics();

			virtual void interpolate(MPCSolution & result, double currentTime, const VelReference & velRef)=0;

		protected:
			virtual void computeOneDynamicMatrices(DynamicMatrix & dyn,
					double S, double T, int N, DynamicMatrixType type)=0;


		protected:
			const MPCData * generalData_;
			const RobotData * robotData_;
			const Interpolation * interpolation_;

			BodyState state_;
			std::vector<DynamicMatrix> pos_vec_;
			std::vector<DynamicMatrix> vel_vec_;
			std::vector<DynamicMatrix> acc_vec_;
			std::vector<DynamicMatrix> jerk_vec_;
			std::vector<DynamicMatrix> cop_vec_;

			DynamicMatrix posInterpol_;
			DynamicMatrix velInterpol_;
			DynamicMatrix accInterpol_;
			DynamicMatrix copInterpol_;

			int matrixNumber_;
	};

}

#endif //RIGID_BODY
