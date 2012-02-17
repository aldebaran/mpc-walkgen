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

				/// \brief compute the value of the polynomial of degree 5 p(x)
				inline double p(const Eigen::Matrix<double,6,1> & factor, double x)  const{
					return factor(0)*pow5(x) + factor(1)*pow4(x) + factor(2)*pow3(x) + factor(3)*pow2(x) + factor(4)*x + factor(5);
				}

				/// \brief compute the value of the derivative of the polynomial of degree 5 p(x)
				inline double dp(const Eigen::Matrix<double,6,1> & factor, double x) const{
					return 5*factor(0)*pow4(x) + 4*factor(1)*pow3(x) + 3*factor(2)*pow2(x) + 2*factor(3)*x + factor(4);
				}

				/// \brief compute the value of the second derivative of the polynomial of degree 5 p(x)
				inline double ddp(const Eigen::Matrix<double,6,1> & factor, double x)  const{
					return 20*factor(0)*pow3(x) + 12*factor(1)*pow2(x) + 6*factor(2)*x + 2*factor(3);
				}

		private:
				Eigen::Matrix3d AinvNorm_;
	};
}

/*! \fn MPCWalkgen::Interpolation::Interpolation(GeneralData * generalData, RigidBodySystem * robot)
* \brief Constructor
*/


#endif //INTERPOLATION
