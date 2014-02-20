////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/constant.h>
#include <mpc-walkgen/tools.h>
#include <cmath>
#include "macro.h"

namespace MPCWalkgen
{
using namespace Eigen;

template <typename Scalar>
void Tools::ConstantJerkDynamic<Scalar>::computeCopDynamic(Scalar S, Scalar T,
                                                   int N, LinearDynamic<Scalar>& dyn,
                                                   Scalar comHeight, Scalar gravityX,
                                                   Scalar gravityZ, Scalar mass,
                                                   Scalar totalMass)
{
  assert(S>0.0);
  assert(T>0.0);
  assert(N>0);
  assert(std::abs(gravityZ)>Constant<Scalar>::EPSILON);
  assert(totalMass>=mass);
  assert(totalMass>Constant<Scalar>::EPSILON);
  assert(mass>=0.0);

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
    dyn.S(i, 2) = m*(0.5f*SS
                  + static_cast<Scalar>(i)*T*S
                  + 0.5f*std::pow(static_cast<Scalar>(i), 2)*TT
                  - comHeight/gravityZ);

    dyn.K(i) = -m*comHeight*gravityX/gravityZ;

    dyn.U(i, 0) = dyn.UT(0, i) = m*(SSS/6.0f
                                    + 0.5f*static_cast<Scalar>(i)*T*SS
                                    + 0.5f*std::pow(static_cast<Scalar>(i), 2)*S*TT
                                    - S*comHeight/gravityZ);
    for(int j=1; j<=i; ++j)
    {
      dyn.U(i, j) = dyn.UT(j, i) = m*TTT*(static_cast<Scalar>(1.0/6.0)
                                          + 0.5f*static_cast<Scalar>(i-j)
                                          + 0.5f*std::pow(static_cast<Scalar>(i-j),2))
                                   - m*T*comHeight/gravityZ;
    }
  }

  Tools::inverseLU(dyn.U, dyn.Uinv, Constant<Scalar>::EPSILON);
  dyn.UTinv = dyn.Uinv.transpose();
}

template <typename Scalar>
void Tools::ConstantJerkDynamic<Scalar>::computePosDynamic(Scalar S, Scalar T,
                                                   int N, LinearDynamic<Scalar>& dyn)
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
    dyn.S(i, 0) = 1.0f;
    dyn.S(i, 1) = static_cast<Scalar>(i)*T + S;
    dyn.S(i, 2) = 0.5f*SS
                + static_cast<Scalar>(i)*T*S
                + 0.5f*std::pow(static_cast<Scalar>(i), 2)*TT;

    dyn.U(i, 0) = dyn.UT(0, i) = SSS/6.0f
                                 + 0.5f*static_cast<Scalar>(i)*T*SS
                                 + 0.5f*std::pow(static_cast<Scalar>(i), 2)*S*TT;
    for(int j=1; j<=i; ++j)
    {
      dyn.U(i, j) = dyn.UT(j, i) = TTT*(static_cast<Scalar>(1.0/6.0)
                                   + 0.5f*static_cast<Scalar>(i-j)
                                   + 0.5f*std::pow(static_cast<Scalar>(i-j),2));
    }

  }

  Tools::inverseLU(dyn.U, dyn.Uinv, Constant<Scalar>::EPSILON);
  dyn.UTinv = dyn.Uinv.transpose();
}

template <typename Scalar>
void Tools::ConstantJerkDynamic<Scalar>::computeVelDynamic(Scalar S, Scalar T,
                                                           int N, LinearDynamic<Scalar>& dyn)
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

    dyn.U(i, 0) = dyn.UT(0, i) = 0.5f*SS + static_cast<Scalar>(i)*T*S;
    for(int j=1; j<=i; ++j)
    {
      dyn.U(i, j) = dyn.UT(j, i) = 0.5f*TT + static_cast<Scalar>(i-j)*TT;
    }
  }

  Tools::inverseLU(dyn.U, dyn.Uinv, Constant<Scalar>::EPSILON);
  dyn.UTinv = dyn.Uinv.transpose();
}

template <typename Scalar>
void Tools::ConstantJerkDynamic<Scalar>::computeOrder2PosDynamic(Scalar S, Scalar T,
                                                                 int N, LinearDynamic<Scalar>& dyn)
{
  assert(S>0.0);
  assert(T>0.0);
  assert(N>0);

  Scalar SS = std::pow(S, 2);
  Scalar TT = std::pow(T, 2);

  dyn.reset(N, 2, N);

  for (int i=0; i<N; ++i)
  {
    dyn.S(i, 0) = 1.0;
    dyn.S(i, 1) = static_cast<Scalar>(i)*T + S;

    dyn.U(i, 0) = dyn.UT(0, i) = 0.5f*SS + static_cast<Scalar>(i)*T*S;
    for(int j=1; j<=i; ++j)
    {
      dyn.U(i, j) = dyn.UT(j, i) = 0.5f*TT + static_cast<Scalar>(i-j)*TT;
    }
  }

  Tools::inverseLU(dyn.U, dyn.Uinv, Constant<Scalar>::EPSILON);
  dyn.UTinv = dyn.Uinv.transpose();
}

template <typename Scalar>
void Tools::ConstantJerkDynamic<Scalar>::computeAccDynamic(Scalar S, Scalar T,
                                                           int N, LinearDynamic<Scalar>& dyn)
{
  assert(std::abs(S)>0.0);
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

template <typename Scalar>
void Tools::ConstantJerkDynamic<Scalar>::computeOrder2VelDynamic(Scalar S, Scalar T,
                                                                 int N, LinearDynamic<Scalar>& dyn)
{
  assert(T>0.0);
  assert(std::abs(S)>0.0);
  assert(N>0);

  dyn.reset(N, 2, N);

  dyn.Uinv(0, 0) = dyn.UTinv(0, 0) = 1/S;

  if(dyn.Uinv.rows()>1)
  {
    dyn.Uinv(1, 0) = dyn.UTinv(0, 1) = -1/S;
  }

  for (int i=0; i<N; ++i)
  {
    dyn.S(i, 1) = 1.0;

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

template <typename Scalar>
void Tools::ConstantJerkDynamic<Scalar>::computeJerkDynamic(int N, LinearDynamic<Scalar>& dyn)
{
  dyn.S.setZero(N, 3);
  dyn.K.setZero(N);
  dyn.U = MatrixX::Identity(N, N);
  dyn.UT = MatrixX::Identity(N, N);
  dyn.Uinv = MatrixX::Identity(N, N);
  dyn.UTinv = MatrixX::Identity(N, N);
}

template <typename Scalar>
void Tools::ConstantJerkDynamic<Scalar>::updateState(Scalar jerk, Scalar T, VectorX& state)
{
  assert(jerk==jerk);
  assert(T>0);

  state(0) += state(1)*T
      + 0.5f*state(2)*std::pow(T, 2)
      + jerk*std::pow(T, 3)/6.0f;

  state(1) += state(2)*T
      + 0.5f*jerk*std::pow(T, 2);

  state(2) += jerk*T;
}

  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(Tools::ConstantJerkDynamic);
}
