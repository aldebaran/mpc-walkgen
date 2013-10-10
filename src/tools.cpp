#include "tools.h"
#include <Eigen/LU>
#include <cmath>

using namespace MPCWalkgen;
using namespace Eigen;

void Tools::ConstantJerkDynamic::computeCopDynamic(Scalar S, Scalar T,
                                                   int N, LinearDynamic& dyn,
                                                   Scalar comHeight, Scalar gravityX,
                                                   Scalar gravityZ, Scalar mass,
                                                   Scalar totalMass)
{
  assert(S>0.0);
  assert(T>0.0);
  assert(N>0);
  assert(std::abs(gravityZ)>EPSILON);
  assert(totalMass>=mass);
  assert(totalMass>EPSILON);
  assert(mass>=0);

  Scalar SS = std::pow(S, 2);
  Scalar SSS = std::pow(S, 3);
  Scalar TT = std::pow(T, 2);
  Scalar TTT = std::pow(T, 3);
  Scalar m = mass/totalMass;

  dyn.reset(N, 3, N);

  for (int i=0; i<N; ++i)
  {
    dyn.S(i, 0) = m;
    dyn.S(i, 1) = m*(static_cast<Scalar>(i)*T + S);
    dyn.S(i, 2) = m*(0.5*SS
                  + static_cast<Scalar>(i)*T*S
                  + 0.5*std::pow(static_cast<Scalar>(i), 2)*TT
                  - comHeight/gravityZ);

    dyn.K(i) = -m*comHeight*gravityX/gravityZ;

    dyn.U(i, 0) = dyn.UT(0, i) = m*(SSS/6.0
                                    + 0.5*static_cast<Scalar>(i)*T*SS
                                    + 0.5*std::pow(static_cast<Scalar>(i), 2)*S*TT
                                    - S*comHeight/gravityZ);
    for(int j=1; j<=i; ++j)
    {
      dyn.U(i, j) = dyn.UT(j, i) = m*TTT*(1.0/6.0
                                          + 0.5*static_cast<Scalar>(i-j)
                                          + 0.5*std::pow(static_cast<Scalar>(i-j),2))
                                   - m*T*comHeight/gravityZ;
    }
  }

  inverseLU(dyn.U, dyn.Uinv, EPSILON);
  dyn.UTinv = dyn.Uinv.transpose();
}

void Tools::ConstantJerkDynamic::computePosDynamic(Scalar S, Scalar T,
                                                   int N, LinearDynamic& dyn)
{
  assert(S>0.0);
  assert(T>0.0);
  assert(N>0);

  Scalar SS = std::pow(S, 2);
  Scalar SSS = std::pow(S, 3);
  Scalar TT = std::pow(T, 2);
  Scalar TTT = std::pow(T, 3);

  dyn.reset(N, 3, N);

  for (int i=0; i<N; ++i)
  {
    dyn.S(i, 0) = 1.0;
    dyn.S(i, 1) = static_cast<Scalar>(i)*T + S;
    dyn.S(i, 2) = 0.5*SS
                + static_cast<Scalar>(i)*T*S
                + 0.5*std::pow(static_cast<Scalar>(i), 2)*TT;

    dyn.U(i, 0) = dyn.UT(0, i) = SSS/6.0
                                 + 0.5*static_cast<Scalar>(i)*T*SS
                                 + 0.5*std::pow(static_cast<Scalar>(i), 2)*S*TT;
    for(int j=1; j<=i; ++j)
    {
      dyn.U(i, j) = dyn.UT(j, i) = TTT*(1.0/6.0
                                   + 0.5*static_cast<Scalar>(i-j)
                                   + 0.5*std::pow(static_cast<Scalar>(i-j),2));
    }

  }

  inverseLU(dyn.U, dyn.Uinv, EPSILON);
  dyn.UTinv = dyn.Uinv.transpose();
}

void Tools::ConstantJerkDynamic::computeVelDynamic(Scalar S, Scalar T,
                                                   int N, LinearDynamic& dyn)
{
  assert(S>0.0);
  assert(T>0.0);
  assert(N>0);

  Scalar SS = std::pow(S, 2);
  Scalar TT = std::pow(T, 2);

  dyn.reset(N, 3, N);

  for (int i=0; i<N; ++i)
  {
    dyn.S(i, 1) = 1.0;
    dyn.S(i, 2) = static_cast<Scalar>(i)*T + S;

    dyn.U(i, 0) = dyn.UT(0, i) = 0.5*SS + static_cast<Scalar>(i)*T*S;
    for(int j=1; j<=i; ++j)
    {
      dyn.U(i, j) = dyn.UT(j, i) = 0.5*TT + static_cast<Scalar>(i-j)*TT;
    }
  }

  inverseLU(dyn.U, dyn.Uinv, EPSILON);
  dyn.UTinv = dyn.Uinv.transpose();
}

void Tools::ConstantJerkDynamic::computeAccDynamic(Scalar S, Scalar T,
                                                   int N, LinearDynamic& dyn)
{
  assert(T>0.0);
  assert(N>0);

  dyn.reset(N, 3, N);

  dyn.Uinv(0, 0) = dyn.UTinv(0, 0) = 1/S;

  if(dyn.Uinv.rows()>1)
  {
  dyn.Uinv(1, 0) = dyn.UTinv(0, 1) = -1/S;
  }

  for (int i=0; i<N; ++i)
  {
    dyn.S(i, 2) = 1.0;

    dyn.U(i, 0) = dyn.UT(0, i) = S;

    for(int j=1; j<=i; ++j)
    {
      dyn.U(i, j) = dyn.UT(j, i) = T;
      if(i == j)
      {
        dyn.Uinv(i, j) = dyn.UTinv(j, i) = 1/T;
      }
      if(i == j+1)
      {
        dyn.Uinv(i, j) = dyn.UTinv(j, i) = -1/T;
      }
    }

  }
}

void Tools::ConstantJerkDynamic::computeJerkDynamic(int N, LinearDynamic& dyn)
{
  dyn.S.setZero(N, 3);
  dyn.K.setZero(N);
  dyn.U = MatrixX::Identity(N, N);
  dyn.UT = MatrixX::Identity(N, N);
  dyn.Uinv = MatrixX::Identity(N, N);
  dyn.UTinv = MatrixX::Identity(N, N);
}

void Tools::ConstantJerkDynamic::updateState(Scalar jerk, Scalar T, VectorX& state)
{
  assert(jerk==jerk);
  assert(T>0);

  state(0) += state(1)*T
      + 0.5*state(2)*std::pow(T, 2)
      + jerk*std::pow(T, 3)/6.0;

  state(1) += state(2)*T
      + 0.5*jerk*std::pow(T, 2);

  state(2) += jerk*T;
}

void Tools::inverseLU(const MatrixX& A, MatrixX& Ap, Scalar eps){
  FullPivLU<MatrixX> lu(A);
  Ap = lu.inverse();
  for(int i=0; i<Ap.rows(); ++i){
    for(int j=0; j<Ap.cols(); ++j){
      if (std::abs(Ap(i,j))<eps){
        Ap(i, j)=0;
      }
    }
  }

}
