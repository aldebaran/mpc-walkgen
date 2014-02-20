////////////////////////////////////////////////////////////////////////////////
///
///\file type.h
///\brief Some structures and typedefs
///\author Lafaye Jory
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_CONVEXPOLYGON_H
#define MPC_WALKGEN_CONVEXPOLYGON_H

#include <mpc-walkgen/api.h>
#include <mpc-walkgen/type.h>

#ifdef _MSC_VER
# pragma warning( push )
// C4251: class needs to have DLL interface
# pragma warning( disable: 4251)
#endif

namespace MPCWalkgen
{
  /// \brief  Define a 2D convex polygon bounded by a set p of vertices
  template <typename Scalar>
  class MPC_WALKGEN_API ConvexPolygon
  {
    TEMPLATE_TYPEDEF(Scalar)
    public:

      ConvexPolygon();

      ConvexPolygon(const vectorOfVector2 &p);

      ~ConvexPolygon();

      inline const vectorOfVector2& getVertices() const
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
      static vectorOfVector2 extractVertices(const vectorOfVector2 &points);
      /// \brief Measure the angle between vec1 and vec2.
      ///        Returns -pi if vec1 = -vec2 and 0 if vec1=0 or vec2=0
      static Scalar angleBetweenVecs(const Vector2& vec1, const Vector2 &vec2);
      /// \brief Return the vector index with the lowest value of Y coordinate.
      ///        If several vectors match this value, it returns the leftmost one
      ///        among these vectors, i.e. the vector with the lowest value of X coordinate.
      ///        All vectors are assumed to have the same Z coordinate.
      static int getIndexOfLowestAndLeftmostVertice(const vectorOfVector2 &p);
      /// \brief Return the next counter-clockwise element of the ConvexPolygon,
      ///        ptot[currentVerticeIndex] being the current convex polygon vertice and
      ///        lastVertice the one before.
      static int getIndexOfSmallestAngleVertice(int currentVerticeIndex,
                                                const Vector2 &lastVertice,
                                                const vectorOfVector2 &ptot);

    private:
      /// \brief Compute Vectors generalConstraintsMatrixCoefsForX_,
      ///        generalConstraintsMatrixCoefsForY_, generalConstraintsConstantPart_,
      ///        and values of xInfBound_, yInfBound_, xSupBound_ and ySupBound_.
      void computeBoundsAndGeneralConstraintValues();

    private:
      /// \brief p_ is the set of vertices of the convex polygon
      vectorOfVector2 p_;

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
}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
