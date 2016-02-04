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
#include <iosfwd>

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

    bool dimensionsAreConsistent(int nbVar, int nbCtr) const;

    /// \brief Normalize all QP matrices except xu and xl using two normalization
    ///        factors computed from Q and A
    void normalizeMatrices(Scalar epsilon);
    /// \brief If the smallest element m of mat is smaller than 1,
    ///        this function returns 1/m. Otherwise it returns 1
    static Scalar getNormalizationFactor(const MatrixXrm& mat, Scalar epsilon);

  public:
    MatrixXrm Q;
    VectorX p;

    VectorX xl;
    VectorX xu;

    MatrixXrm A;
    VectorX bu;
    VectorX bl;
  };

  template <typename Scalar>
  class QPSolver
  {

  public:
    virtual ~QPSolver() {}
    // Return true if a solution is found. In such a case, the solution is
    // written to sol. If no solution is found, sol may still be written to.
    //
    // if useWarmStart==true, and the solver was already initialized,
    // then m.Q and m.A won't be used.
    //
    // If no solution is found and os != nullptr, then debug information may
    // be written to *os.
    virtual bool solve(const QPMatrices<Scalar>& m,
                       int maxNbIterations,
                       typename QPMatrices<Scalar>::VectorX& sol,
                       bool useWarmStart = false,
                       std::ostream *os = nullptr) = 0;
    virtual int getNbVar() const = 0;
    virtual int getNbCtr() const = 0;
  };

///QPMatrices
template <typename Scalar>
bool QPMatrices<Scalar>::dimensionsAreConsistent(int nbVar, int nbCtr) const {
  return (Q.rows() == nbVar) && (Q.cols() == nbVar) && (p.size() == nbVar) &&
      (xl.size() == nbVar) && (xu.size() == nbVar) &&
      (A.cols() == nbVar) && (A.rows() == nbCtr) &&
      (bl.size() == nbCtr) && (bu.size() == nbCtr);
}

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
