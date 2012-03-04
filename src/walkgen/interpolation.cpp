#include <mpc-walkgen/interpolation.h>

using namespace MPCWalkgen;
using namespace Eigen;

Interpolation::Interpolation()
{
	AinvNorm_(0,0) = 6  ;  AinvNorm_(0,1) = -3  ;  AinvNorm_(0,2) = 0.5;
	AinvNorm_(1,0) = -15;  AinvNorm_(1,1) = 7   ;  AinvNorm_(1,2) = -1;
	AinvNorm_(2,0) = 10 ;  AinvNorm_(2,1) = -4  ;  AinvNorm_(2,2) = 0.5;
}

Interpolation::~Interpolation(){}


void Interpolation::computeInterpolationByJerk(VectorXd &solutionX, VectorXd &solutionY, const BodyState &state,
		 const LinearDynamics &dyn, double jerkX, double jerkY) const{

	int nbSamples = dyn.U.cols();
	VectorXd UX = VectorXd::Constant(nbSamples, jerkX);
	VectorXd UY = VectorXd::Constant(nbSamples, jerkY);

	solutionX = dyn.S*state.x + dyn.U*UX;
	solutionY = dyn.S*state.y + dyn.U*UY;
}


void Interpolation::computePolynomialNormalisedFactors( Eigen::Matrix<double,6,1> &factor,
		const Vector3d &initialstate, const Vector3d &finalState, double T) const
{
	Vector3d b;

	factor(5) = initialstate(0);
	factor(4) = T*initialstate(1);
	factor(3) = T*T*initialstate(2)/2;

	b(0) = finalState(0) - factor(5) - factor(4) - factor(3);
	b(1) = finalState(1) - factor(4) - 2*factor(3);
	b(2) = finalState(2) - 2*factor(3);

	Vector3d abc;

	abc=AinvNorm_*b;


	factor(2) = abc(2);
	factor(1) = abc(1);
	factor(0) = abc(0);
}

void Interpolation::computePolynomialFactors( Eigen::Matrix<double,6,1>  & factor,
	const Vector3d & initialstate, const Vector3d & finalState, double T ) const
{
	Matrix3d Ainv;
	Vector3d b;

	Ainv(0,0) = 6/pow5(T)  ;  Ainv(0,1) = -3/pow4(T)  ;  Ainv(0,2) = 1/(2*pow2(T));
	Ainv(1,0) = -15/pow4(T);  Ainv(1,1) = 7/pow3(T)   ;  Ainv(1,2) = -1/pow2(T);
	Ainv(2,0) = 10/pow3(T) ;  Ainv(2,1) = -4/pow2(T)  ;  Ainv(2,2) = 1/(2*T);

	b(0) = finalState(0) - initialstate(0) - initialstate(1)*T - initialstate(2)*T*T/2;
	b(1) = finalState(1) - initialstate(1) - initialstate(2)*T;
	b(2) = finalState(2) - initialstate(2);

	Vector3d abc;
	abc=Ainv*b;

	factor(5) = initialstate(0);
	factor(4) = initialstate(1);
	factor(3) = initialstate(2)/2;
	factor(2) = abc(2);
	factor(1) = abc(1);
	factor(0) = abc(0);
}

