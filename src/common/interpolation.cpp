#include "interpolation.h"

using namespace MPCWalkgen;
using namespace Eigen;

Interpolation::Interpolation()
  :AinvNorm_(9,9)
  ,b_(9)
  ,abc_(9)
{
  AinvNorm_<< 9./2. , 3./2.,  1./6. ,  9./2. ,  0.   , -1./12.,  9./2. , -3./2.,  1./6.  ,
             -9.    ,-3./2.,  5./12., -9.    ,  3./2.,  5./12., -9.    ,  9./2., -7./12. ,
              27./2., 3.   , -3./4. ,  27./2., -3./2., -1./2. ,  27./2., -6.   ,  3./4.  ,
             -9./2. ,-2.   ,  5./12., -9./2. ,  1./2.,  1./6. , -9./2. ,  2.   , -1./4.  ,
             -1./2. , 4./9., -7./108., 1./2. , -1./18.,-1./54.,  1./2. , -2./9.,  1./36. ,
              9./2. , 0.   , -1./12.,  9./2. , -3./2.,  1./6. ,  9./2. , -3.   ,  11./12.,
             -27./2., 0.   ,  1./4. , -27./2.,  9./2., -1./2. , -27./2.,  9.   , -9./4.  ,
              27./2., 0.   , -1./4. ,  27./2., -9./2.,  1./2. ,  27./2., -8.   ,  7./4.  ,
             -9./2. , 0.   ,  1./12., -9./2. ,  3./2., -1./6. , -7./2. ,  2.   , -5./12. ;
}

Interpolation::~Interpolation(){}


void Interpolation::computeInterpolationByJerk(VectorXd &solutionX, VectorXd &solutionY,
                                               const BodyState &state, const LinearDynamics &dyn,
                                               double jerkX, double jerkY) const{

  int nbSamples = dyn.U.cols();
  VectorXd UX = VectorXd::Constant(nbSamples, jerkX);
  VectorXd UY = VectorXd::Constant(nbSamples, jerkY);

  solutionX = dyn.S*state.x + dyn.U*UX;
  solutionY = dyn.S*state.y + dyn.U*UY;
}

void Interpolation::computeInterpolationByJerk(VectorXd &solution, const VectorXd &state,
                                               const LinearDynamics &dyn, double jerk) const{

  int nbSamples = dyn.U.cols();
  VectorXd U = VectorXd::Constant(nbSamples, jerk);
  solution = dyn.S*state + dyn.U*U;
}


void Interpolation::computePolynomialNormalisedFactors( Eigen::VectorXd &factor,
                                                        const Vector3d &initialstate,
                                                        const Vector3d &finalState,
                                                        double T) const{
  factor(3) = initialstate(0);
  factor(2) = T*initialstate(1);
  factor(1) = T*T*initialstate(2)/2;

  b_(0) = - T*T*initialstate(2)/18 - T*initialstate(1)/3 - initialstate(0);
  b_(1) = - T*T*initialstate(2)/3 - T*initialstate(1);
  b_(2) = - T*T*initialstate(2);
  b_(3) = 0;
  b_(4) = 0;
  b_(5) = 0;
  b_(6) = finalState(0);
  b_(7) = T*finalState(1);
  b_(8) = T*T*finalState(2);

  abc_=AinvNorm_*b_;

  factor(0) = abc_(0);
  factor(4) = abc_(1);
  factor(5) = abc_(2);
  factor(6) = abc_(3);
  factor(7) = abc_(4);
  factor(8) = abc_(5);
  factor(9) = abc_(6);
  factor(10) = abc_(7);
  factor(11) = abc_(8);
}

void Interpolation::selectFactors(Eigen::Vector4d & subfactor,
                             const Eigen::VectorXd & factor,
                             double t, double T) const{
  if (t<=T/3){
    subfactor[0] = factor[0];
    subfactor[1] = factor[1];
    subfactor[2] = factor[2];
    subfactor[3] = factor[3];
  }else if(t<=2*T/3){
    subfactor[0] = factor[4];
    subfactor[1] = factor[5];
    subfactor[2] = factor[6];
    subfactor[3] = factor[7];
  }else{
    subfactor[0] = factor[8];
    subfactor[1] = factor[9];
    subfactor[2] = factor[10];
    subfactor[3] = factor[11];
  }
}
