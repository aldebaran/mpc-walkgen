////////////////////////////////////////////////////////////////////////////////
///
///\file qpsolver.h
///\brief Interface template for a QP solver
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_QPSOLVER_H
#define MPC_WALKGEN_QPSOLVER_H

#include <Eigen/Core>

namespace MPCWalkgen
{

  // matrices which describe a QP
  // x = argmin 0.5 * x' * Q * x + p'*x
  //
  // such that
  //    xl <= x <= xu
  //    bl <= A*x <= bu
  // Q is called the hessian
  // p is called the gradient
  //
  // The QP solver we use expects matrices in row-major order.
  // Note that this does not matter for the Q matrix, which is symmetric.
  template <typename Scalar>
  class QPMatrices
  {
  public:
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXrm;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;

    /// \brief Normalize all QP matrices except xu and xl using two normalization
    ///        factors computed from Q and A
    void normalizeMatrices(Scalar epsilon);
    /// \brief If the smallest element m of mat is smaller than 1,
    ///        this function returns 1/m. Otherwise it returns 1
    static Scalar getNormalizationFactor(const MatrixXrm& mat, Scalar epsilon);

  public:
    MatrixXrm Q;
    VectorX p;

    MatrixXrm A;
    VectorX bu;
    VectorX bl;
    VectorX xl;
    VectorX xu;
  };

  template <typename Scalar>
  class QPSolver
  {

  public:
    virtual ~QPSolver() {}
    virtual bool solve(const QPMatrices<Scalar>& m,
                       typename QPMatrices<Scalar>::VectorX& sol,
                       bool useWarmStart = false) = 0;
    virtual int getNbVar() const = 0;
    virtual int getNbCtr() const = 0;
  };

///QPMatrices
template <typename Scalar>
void QPMatrices<Scalar>::normalizeMatrices(Scalar epsilon)
{
  Scalar objFactor = getNormalizationFactor(Q, epsilon);
  Scalar ctrFactor = getNormalizationFactor(A, epsilon);

  Q *= objFactor;
  p *= objFactor;

  A *= ctrFactor;
  bu *= ctrFactor;
  bl *= ctrFactor;
}

template <typename Scalar>
Scalar QPMatrices<Scalar>::getNormalizationFactor(const MatrixXrm& mat,
                                                  Scalar epsilon)
{
  Scalar normalizationFactor = 1.0;
  for(int i=0; i<mat.rows(); ++i)
  {
    for(int j=0; j<mat.cols(); ++j)
    {
      Scalar v = std::abs(mat(i, j));
      if (v>epsilon && v<normalizationFactor)
      {
        normalizationFactor = v;
      }
    }
  }

  return 1/normalizationFactor;
}
}
#endif
