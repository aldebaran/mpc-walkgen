#ifndef INTERPOLATION
#define INTERPOLATION

////////////////////////////////////////////////////////////////////////////////
///
///\file	interpolation.h
///\brief	A tools class wich provide some interpolation methods
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author	Keith François
///\version	1.0
///\date	05/01/12
///
////////////////////////////////////////////////////////////////////////////////



#include <mpc-walkgen/types.h>
#include <mpc-walkgen/tools.h>
#include <Eigen/Dense>

namespace MPCWalkgen{

	class Interpolation{

		public:
				Interpolation();
				~Interpolation();

				void computeInterpolationByJerk(Eigen::VectorXd & solutionX, Eigen::VectorXd & solutionY, const BodyState & state,
						 const DynamicMatrix & dyn, double jerkX, double jerkY) const;

				void computePolynomialNormalisedFactors(
						Eigen::Matrix<double,6,1> & factor, const Eigen::Vector3d & initialstate,
						const Eigen::Vector3d & finalState, double T ) const;

				void computePolynomialFactors(
						Eigen::Matrix<double,6,1>  & factor,
						const Eigen::Vector3d & initialstate,
						const Eigen::Vector3d & finalState, double T ) const;
		private:
				Eigen::Matrix3d AinvNorm_;
	};
}

/*! \fn MPCWalkgen::Interpolation::Interpolation(GeneralData * generalData, RigidBodySystem * robot)
* \brief Constructor
*/


#endif //INTERPOLATION
