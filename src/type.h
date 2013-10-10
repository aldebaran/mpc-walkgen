////////////////////////////////////////////////////////////////////////////////
///
///\file type.h
///\brief Some structures and typedefs
///\author Lafaye Jory
///\author de Gourcuff Martin
///\date 19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_TYPE_H
#define MPC_WALKGEN_TYPE_H

#include <vector>
#include <cassert>
#include <iostream>
#include <mpc-walkgen/shared_type.h>

namespace MPCWalkgen
{
#define QPOASES_REAL_IS_FLOAT

  static const Scalar EPSILON = 0.0001;
  static const Scalar GRAVITY_NORM = 9.81;
  static const Vector3 GRAVITY_VECTOR = Vector3(0, 0, 9.81);
  static const Scalar MAXIMUM_BOUND_VALUE = 10e10;

  /// \brief  Matrices relative to a linear dynamic of type : Y = U X + S x + K
  ///         Where X is the control variables and x the initial state
  ///         UT is U transpose.
  ///         K is a constant which does not depend on the initial state. For now it is
  ///         always a zero vector, except for dynamics related to the CoP.
  ///         If U is square, Uinv and UTinv are its inverse and transpose inverse.
  ///         inverse, respectively. If U is not square, these matrices are filled
  ///         with NaN values.
  ///         -nbSamples stands for Y's number of rows
  ///         -stateVectorSize stands for x's number of rows
  ///         -variableVectorSize stands for X's number of rows
  class LinearDynamic
  {
    public:
      void reset(int nbSamples,
                 int stateVectorSize,
                 int variableVectorSize);

    public:
      MatrixX U;
      MatrixX UT;
      MatrixX Uinv;
      MatrixX UTinv;
      MatrixX S;
      VectorX K;
  };


  class QPMatrices
  {
    public:
      /// \brief Normalize all QP matrices except xu and xl using two normalization
      ///        factors computed from Q and A
      void normalizeMatrices();
      /// \brief If the smallest element m of mat is smaller than 1,
      ///        this function returns 1/m. Otherwise it returns 1
      static Scalar getNormalizationFactor(const MatrixX& mat);

    public:
      MatrixX Q;
      VectorX p;

      MatrixX A;
      MatrixX At;
      VectorX bu;
      VectorX bl;
      VectorX xl;
      VectorX xu;
  };

  /// \brief  Define a 2D convex polygon bounded by a set p of vertices
  class ConvexPolygon
  {
    public:

      ConvexPolygon();

      ConvexPolygon(std::vector<Vector2> p);

      ~ConvexPolygon();

      inline const std::vector<Vector2>& getVertices() const
      {return p_;}

      /// \brief Get the number of vertices of the convex polygon
      inline const int getNbVertices() const
      {return p_.size();}

      inline Scalar getXSupBound() const
      {return xSupBound_;}
      inline Scalar getXInfBound() const
      {return xInfBound_;}
      inline Scalar getYSupBound() const
      {return ySupBound_;}
      inline Scalar getYInfBound() const
      {return yInfBound_;}

      inline const VectorX& getGeneralConstraintsMatrixCoefsForX() const
      {return generalConstraintsMatrixCoefsForX_;}
      inline const VectorX& getGeneralConstraintsMatrixCoefsForY() const
      {return generalConstraintsMatrixCoefsForY_;}
      inline const VectorX& getGeneralConstraintsConstantPart() const
      {return generalConstraintsConstantPart_;}

      inline int getNbGeneralConstraints() const
      {return generalConstraintsConstantPart_.rows();}

      //TODO: think about changing type and tools into something else. E.g. angleBetweenVecs
      //should be in tools, but tools depends of type... Also these static attributes are an
      //ugly solution but comfortable for now.


      /// \brief Return the set of vertices of the convex polygon of all vectors from Vec
      ///        The algorithm is similar to the gift-wrapping or Jarvis
      ///        march algorithm. The convex polygon vertices are counter-clockwise ordered
      static std::vector<Vector2> extractVertices(const std::vector<Vector2>& points);
      /// \brief Measure the angle between vec1 and vec2.
      ///        Returns -pi if vec1 = -vec2 and 0 if vec1=0 or vec2=0
      static Scalar angleBetweenVecs(const Vector2& vec1, const Vector2& vec2);
      /// \brief Return the vector index with the lowest value of Y coordinate.
      ///        If several vectors match this value, it returns the leftmost one
      ///        among these vectors, i.e. the vector with the lowest value of X coordinate.
      ///        All vectors are assumed to have the same Z coordinate.
      static int getIndexOfLowestAndLeftmostVertice(const std::vector<Vector2> &p);
      /// \brief Return the next counter-clockwise element of the ConvexPolygon,
      ///        ptot[currentVerticeIndex] being the current convex polygon vertice and
      ///        lastVertice the one before.
      static int getIndexOfSmallestAngleVertice(int currentVerticeIndex,
                                                const Vector2 &lastVertice,
                                                const std::vector<Vector2> &ptot);

    private:
      /// \brief Compute Vectors generalConstraintsMatrixCoefsForX_,
      ///        generalConstraintsMatrixCoefsForY_, generalConstraintsConstantPart_,
      ///        and values of xInfBound_, yInfBound_, xSupBound_ and ySupBound_.
      void computeBoundsAndGeneralConstraintValues();

    private:
      /// \brief p_ is the set of vertices of the convex polygon
      std::vector<Vector2> p_;

      /// \brief Let X be a point of coordinates (x, y). X is located inside the convex polygon
      ///        if it is a solution of the general constraints matrix inequation:
      ///        AX + b <=0
      ///
      ///        Let (p(1),...,p(n)) be the set of vertices of the convex polygon. X is located
      ///        inside the convex polygon if for all i in [1, n],
      ///        (py(i+1) - py(i))(x - px(i)) - (px(i+1) - px(i))(y - py(i)) <= 0
      ///        This is how matrices A and b are built. However, in order to save computation
      ///        time, it is easier to detect bounds contraints of the form Xinf <= X <= Xmax.
      ///        We do have bounds constraints if one or several edges of the convex polygon
      ///        are horizontal or vertical.
      ///
      ///        For a given convex polygon, we store these matrices as below:
      ///        A = (generalConstraintsMatrixCoefsForX_, generalConstraintsMatrixCoefsForY_)
      ///        b = generalConstraintsConstantPart_
      ///        Xinf = (xInfBound_, yInfBound_)
      ///        XSup = (xSupBound_, ySupBound_)
      ///

      Scalar xSupBound_;
      Scalar xInfBound_;
      Scalar ySupBound_;
      Scalar yInfBound_;

      VectorX generalConstraintsMatrixCoefsForX_;
      VectorX generalConstraintsMatrixCoefsForY_;
      VectorX generalConstraintsConstantPart_;

  };

  class Interpolator
  {
    public:
      Interpolator();
      ~Interpolator();

      /// \brief Computes normalised spline factors for each of its three polynoms
      ///        (of degree three). Constraints are at both ends of the spline,
      ///        and at the two junction between the three polynoms
      void computePolynomialNormalisedFactors(VectorX &factor,
                                              const Vector3 & initialstate,
                                              const Vector3 & finalState,
                                              Scalar T ) const;

      /// \brief select the proper 4 factors corresponding to one of the three polynoms in a
      ///        normalised spline of degree three
      void selectFactors(Vector4 & subfactor,
                         const VectorX & factor,
                         Scalar t,
                         Scalar T) const;
    private:
      /// \brief We have a cubic spline of three polynoms with 3 constraints
      ///        (position, velocity and acceleration) on both edges of the spline,
      ///        and 3 constraints (position velocity and acceleration) for each junctions
      ///        between the three polynoms. The spline is normalised, and the junction are
      ///        set at x=1/3 and x=2/3. We have then 2*3 + 2*3 = 12 equations
      ///        (one per constraint), and 12 factor (4 for each polynom) to compute.
      ///        As we have 3 constraints at x=0 for the first polynom, one can easily compute
      ///        the three first factors. The 9 left factors can be obtained from the 9
      ///        equations corresponding to the 9 left constraints. This set of 9 linear
      ///        equations can be written as a matricial equation AX = b_. AinvNorm_ is
      ///        then the inverse of matrix A.
      MatrixX AinvNorm_;
      mutable VectorX b_;
      mutable VectorX abc_;
  };

}

#endif //MPC_WALKGEN_TYPE_H
