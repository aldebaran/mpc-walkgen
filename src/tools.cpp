#include "tools.h"
#include <cmath>

using namespace MPCWalkgen;

void Tools::ConstantJerkDynamic::computeCopDynamic(Scalar T, int N, LinearDynamic& dyn,
                                                   Scalar comHeight, Scalar gravityX,
                                                   Scalar gravityY, Scalar mass,
                                                   Scalar totalMass)
{
  assert(T>0.0);
  assert(N>0);
  assert(std::abs(gravityY)>EPSILON);
  assert(totalMass>=mass);
  assert(totalMass>EPSILON);
  assert(mass>=0);

  Scalar TT = std::pow(T, 2);
  Scalar TTT = std::pow(T, 3);
  Scalar m = mass/totalMass;

  dyn.reset(N);

  for (int i=0; i<N; ++i)
  {
    dyn.S(i,0) = dyn.ST(0,i) = m;
    dyn.S(i,1) = dyn.ST(1,i) = m*(i+1)*T;
    dyn.S(i,2) = dyn.ST(2,i) = m*TT*(1.0/2.0 + i + std::pow(static_cast<Scalar>(i), 2)/2.0)
                             - m*comHeight/gravityY;
    dyn.S(i,3) = dyn.ST(3,i) = -m*comHeight*gravityX/gravityY;

    for(int j=0; j<=i; j++)
    {
      dyn.U(i,j) = dyn.UT(j,i) = m*TTT*
                                 (1.0/6.0 + (i-j)/2.0 + std::pow(static_cast<Scalar>(i-j),2)/2.0)
                               - m*T*comHeight/gravityY;
    }

  }
}

void Tools::ConstantJerkDynamic::computePosDynamic(Scalar T, int N, LinearDynamic& dyn)
{
  assert(T>0.0);
  assert(N>0);

  Scalar TT = std::pow(T, 2);
  Scalar TTT = std::pow(T, 3);

  dyn.reset(N);

  for (int i=0; i<N; ++i)
  {
    dyn.S(i,0) = dyn.ST(0,i) = 1.0;
    dyn.S(i,1) = dyn.ST(1,i) = (i+1)*T;
    dyn.S(i,2) = dyn.ST(2,i) = TT*(1.0/2.0 + i + std::pow(static_cast<Scalar>(i), 2)/2.0);

    for(int j=0; j<=i; j++)
    {
      dyn.U(i,j) = dyn.UT(j,i) =
                  TTT*(1.0/6.0 + (i-j)/2.0 + std::pow(static_cast<Scalar>(i-j),2)/2.0);
    }

  }
}

void Tools::ConstantJerkDynamic::computeVelDynamic(Scalar T, int N,
                                                   LinearDynamic& dyn)
{
  assert(T>0.0);
  assert(N>0);

  Scalar TT = std::pow(T, 2);

  dyn.reset(N);

  for (int i=0; i<N; ++i)
  {
    dyn.S(i,1) = dyn.ST(1,i) = 1.0;
    dyn.S(i,2) = dyn.ST(2,i) = (i+1)*T;

    for(int j=0; j<=i; j++)
    {
      dyn.U(i,j) = dyn.UT(j,i) = TT*(1.0/2.0 + (i-j));
    }

  }
}

void Tools::ConstantJerkDynamic::computeAccDynamic(MPCWalkgen::Scalar T, int N,
                                                   MPCWalkgen::LinearDynamic& dyn)
{
  assert(T>0.0);
  assert(N>0);

  dyn.reset(N);

  for (int i=0; i<N; ++i)
  {
    dyn.S(i,2) = 1;

    for(int j=0; j<=i; j++)
    {
        dyn.U(i,j) = dyn.UT(j,i) = T;
    }

  }
}

void Tools::ConstantJerkDynamic::computeJerkDynamic(int N, LinearDynamic& dyn)
{
  dyn.S.setZero(N,4);
  dyn.ST.setZero(4,N);
  dyn.U = MatrixX::Identity(N,N);
  dyn.UT = MatrixX::Identity(N,N);
}

void Tools::ConstantJerkDynamic::updateState(Scalar jerk, Scalar T, VectorX& state)
{
  assert(jerk==jerk);
  assert(T>0);

  state(0) += state(1)*T
            + state(2)*std::pow(T,2)/2.0
            + jerk*std::pow(T,3)/6.0;

  state(1) += state(2)*T
            + jerk*std::pow(T,2)/2.0;

  state(2) += jerk*T;
}
