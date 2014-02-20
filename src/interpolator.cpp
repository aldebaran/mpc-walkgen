#include <mpc-walkgen/interpolator.h>
#include "macro.h"

namespace MPCWalkgen
{
 ///Interpolator
  template <typename Scalar>
  Interpolator<Scalar>::Interpolator()
    :aInvNorm_(9,9)
    ,b_(9)
    ,abc_(9)
  {
    // temporary to avoid warning about double to float lossy conversion.
    Eigen::Matrix<double, 9,9>AinvNorm;
    AinvNorm << 9./2. , 3./2.,  1./6. ,  9./2. ,  0.   , -1./12.,  9./2. , -3./2.,  1./6.  ,
               -9.    ,-3./2.,  5./12., -9.    ,  3./2.,  5./12., -9.    ,  9./2., -7./12. ,
                27./2., 3.   , -3./4. ,  27./2., -3./2., -1./2. ,  27./2., -6.   ,  3./4.  ,
               -9./2. ,-2.   ,  5./12., -9./2. ,  1./2.,  1./6. , -9./2. ,  2.   , -1./4.  ,
               -1./2. , 4./9., -7./108., 1./2. , -1./18.,-1./54.,  1./2. , -2./9.,  1./36. ,
                9./2. , 0.   , -1./12.,  9./2. , -3./2.,  1./6. ,  9./2. , -3.   ,  11./12.,
               -27./2., 0.   ,  1./4. , -27./2.,  9./2., -1./2. , -27./2.,  9.   , -9./4.  ,
                27./2., 0.   , -1./4. ,  27./2., -9./2.,  1./2. ,  27./2., -8.   ,  7./4.  ,
               -9./2. , 0.   ,  1./12., -9./2. ,  3./2., -1./6. , -7./2. ,  2.   , -5./12. ;
    aInvNorm_ = AinvNorm.cast<Scalar>();
  }

  template <typename Scalar>
  Interpolator<Scalar>::~Interpolator(){}

  template <typename Scalar>
  void Interpolator<Scalar>::computePolynomialNormalisedFactors(
      VectorX &factor,
      const Vector3 &initialstate,
      const Vector3 &finalState,
      Scalar T ) const
  {
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

    abc_=aInvNorm_*b_;

    factor(0) = abc_(0);
    factor.template segment<8>(4) = abc_.template segment<8>(1);
  }

  template <typename Scalar>
  void Interpolator<Scalar>::selectFactors(Vector4 &subfactor,
                                           const VectorX &factor,
                                           Scalar t,
                                           Scalar T) const
  {
    if (t<=T/3.0)
    {
      subfactor = factor.template segment<4>(0);
    }
    else if(t<=2.0*T/3.0)
    {
      subfactor = factor.template segment<4>(4);
    }
    else
    {
      subfactor = factor.template segment<4>(8);
    }
  }

  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(Interpolator);
}
