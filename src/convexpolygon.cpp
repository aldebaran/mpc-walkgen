////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/convexpolygon.h>
#include <mpc-walkgen/constant.h>
#include <boost/math/constants/constants.hpp>
#include "macro.h"

namespace MPCWalkgen
{
  ///Convex Polygon
  template <typename Scalar>
  ConvexPolygon<Scalar>::ConvexPolygon()
    :p_(0)
    ,xSupBound_(Constant<Scalar>::MAXIMUM_BOUND_VALUE)
    ,xInfBound_(-Constant<Scalar>::MAXIMUM_BOUND_VALUE)
    ,ySupBound_(Constant<Scalar>::MAXIMUM_BOUND_VALUE)
    ,yInfBound_(-Constant<Scalar>::MAXIMUM_BOUND_VALUE)
    ,generalConstraintsMatrixCoefsForX_()
    ,generalConstraintsMatrixCoefsForY_()
    ,generalConstraintsConstantPart_()
  {
    computeBoundsAndGeneralConstraintValues();
  }

  template <typename Scalar>
  ConvexPolygon<Scalar>::ConvexPolygon(const vectorOfVector2 &p)
    :p_(extractVertices(p))
    ,xSupBound_(Constant<Scalar>::MAXIMUM_BOUND_VALUE)
    ,xInfBound_(-Constant<Scalar>::MAXIMUM_BOUND_VALUE)
    ,ySupBound_(Constant<Scalar>::MAXIMUM_BOUND_VALUE)
    ,yInfBound_(-Constant<Scalar>::MAXIMUM_BOUND_VALUE)
    ,generalConstraintsMatrixCoefsForX_()
    ,generalConstraintsMatrixCoefsForY_()
    ,generalConstraintsConstantPart_()
  {
    computeBoundsAndGeneralConstraintValues();
  }

  template <typename Scalar>
  ConvexPolygon<Scalar>::~ConvexPolygon(){}

  template <typename Scalar>
  typename Type<Scalar>::vectorOfVector2 ConvexPolygon<Scalar>::extractVertices(
      const vectorOfVector2& points)
  {
    vectorOfVector2 p;

    // Here we handle the degenerate case for which vector vertices is empty
    if(points.empty())
    {
      return p;
    }

    p.reserve(points.size());

    // Adding the first vertex
    int index = getIndexOfLowestAndLeftmostVertice(points);

    // Adding the first point
    p.push_back(points[index]);

    // Finding the index of the second vertex
    index = getIndexOfSmallestAngleVertice(index,
                                           Vector2(points[index](0) - 1,
                                                   points[index](1)),
                                           points);

    // Adding the other vertices until we reach the first index, which
    // means the convex polygon has been completed
    while(index != getIndexOfLowestAndLeftmostVertice(points))
    {
      assert(p.size() <= points.size());

      p.push_back(points[index]);
      index = getIndexOfSmallestAngleVertice(index, *(p.end()-2), points);
    }

    return p;
  }


  template <typename Scalar>
  Scalar ConvexPolygon<Scalar>::angleBetweenVecs(const Vector2& vec1, const Vector2& vec2)
  {
    if(vec1.isZero() ||
       vec2.isZero() ||
       vec1.isApprox(vec2, Constant<Scalar>::EPSILON))
    {
      return static_cast<Scalar>(0.);
    }
    else if (vec1.isApprox(-vec2, Constant<Scalar>::EPSILON))
    {
      return boost::math::constants::pi<Scalar>();
    }
    else
    {
      Scalar angle = vec1.dot(vec2)/(vec1.norm()*vec2.norm());

      if(vec1(0)*vec2(1) - vec1(1)*vec2(0) >= static_cast<Scalar>(0.0))
      {
        return std::acos(angle);
      }
      else
      {
        return -std::acos(angle);
      }
    }
  }

  template <typename Scalar>
  int ConvexPolygon<Scalar>::getIndexOfLowestAndLeftmostVertice(const vectorOfVector2& p)
  {
    //Storing indexes of the vertices with the lowest value of Y coordinate
    std::vector<int> lowestVerticesIndexes;
    lowestVerticesIndexes.reserve(p.size());

    lowestVerticesIndexes.push_back(0);

    for (size_t i=1; i<p.size(); ++i)
    {
      if(p[i](1)<p[lowestVerticesIndexes[0]](1))
      {
        lowestVerticesIndexes.erase(lowestVerticesIndexes.begin(),
                                    lowestVerticesIndexes.end());
        lowestVerticesIndexes.push_back(i);
      }
      else if(std::abs(p[i](1)-p[lowestVerticesIndexes[0]](1)) < Constant<Scalar>::EPSILON)
      {
        lowestVerticesIndexes.push_back(i);
      }
    }

    //Choosing the index of the leftmost vertex among p[lowestVerticesIndexes]
    //i.e. the one with the lowest value of X coordinate
    if (lowestVerticesIndexes.size()>1)
    {
      int index = lowestVerticesIndexes[0];
      for(size_t i=1; i<lowestVerticesIndexes.size(); ++i)
      {
        if(p[lowestVerticesIndexes[i]](0)<p[index](0))
        {
          index = lowestVerticesIndexes[i];
        }
      }
      assert(index < static_cast<int>(p.size()));
      return index;
    }
    else
    {
      assert(lowestVerticesIndexes[0] <static_cast<int>(p.size()));
      return lowestVerticesIndexes[0];
    }
  }


  template <typename Scalar>
  int ConvexPolygon<Scalar>::getIndexOfSmallestAngleVertice(
      int currIndex,
      const Vector2& lastVertice,
      const vectorOfVector2& ptot)
  {
    //Creating vector of indexes of the candidate for the next vertex
    std::vector<int> indexesTab;
    indexesTab.reserve(ptot.size() - 1);

    // We push back index 0 arbitrarily, unless currIndex=0
    if(currIndex!=0)
    {
      indexesTab.push_back(0);
    }
    else
    {
      indexesTab.push_back(1);
    }

    for (size_t i=1; i<ptot.size(); ++i)
    {
      // We do not check the current vertex. This if statement also avoids copies
      // of the current vertex among ptot vector, i.e. we do not count the current
      // vertex more than once
      if(!ptot[i].isApprox(ptot[currIndex], Constant<Scalar>::EPSILON))
      {
        if(angleBetweenVecs(ptot[currIndex] - lastVertice,
                            ptot[i] - ptot[currIndex]) <
           angleBetweenVecs(ptot[currIndex] -
                            lastVertice, ptot[indexesTab[0]] - ptot[currIndex]))
        {
          indexesTab.erase(indexesTab.begin(), indexesTab.end());
          indexesTab.push_back(i);
        }
        else if(std::abs(angleBetweenVecs(ptot[currIndex] - lastVertice,
                                          ptot[i] - ptot[currIndex]) -
                         angleBetweenVecs(ptot[currIndex] - lastVertice,
                                          ptot[indexesTab[0]] - ptot[currIndex]))
               < Constant<Scalar>::EPSILON)
        {
          indexesTab.push_back(i);
        }
      }
    }

    //Choosing the index of the furthest point from the current point
    //among indexesTab
    if (indexesTab.size()>1)
    {
      int index = indexesTab[0];
      for(size_t i=1; i<indexesTab.size(); ++i)
      {
        if((ptot[indexesTab[i]] - ptot[currIndex]).norm() >
           (ptot[index] - ptot[currIndex]).norm())
        {index = indexesTab[i];}
      }
      return index;
    }
    else
    {
      return indexesTab[0];
    }
  }

  template <typename Scalar>
  void ConvexPolygon<Scalar>::computeBoundsAndGeneralConstraintValues()
  {
    // We check whether or not the ith convex polygon side is parallel to axis X or Y.
    // If so, it is stored in bounds constraints vectors. If not, it is a general constraint.
    // One should remember that vertices are ordered counter-clockwise.
    // Note that if two vertices are closer in norm than EPSILON, one of them is discarded
    // in extractVertices(). The constructor then ensures that such a case cannot happen
    // in this function.
    int nbVertices = p_.size();
    for (int i=0; i<nbVertices; ++i)
    {
      Vector2 hullEdge = p_[(i+1)%nbVertices] - p_[i];

      if(std::abs(hullEdge(0))<Constant<Scalar>::EPSILON)
      {
        if(hullEdge(1)<0)
        {
          xInfBound_ = p_[i](0);
        }
        else
        {
          xSupBound_ = p_[i](0);
        }
      }
      else if(std::abs(hullEdge(1))<Constant<Scalar>::EPSILON)
      {
        if(hullEdge(0)<0)
        {
          ySupBound_ = p_[i](1);
        }
        else
        {
          yInfBound_ = p_[i](1);
        }
      }
      else
      {
        int nbElements = generalConstraintsMatrixCoefsForX_.rows();
        //TODO: template class to set _MaxRows for these 3 VectorX and to
        //avoid reallocations
        generalConstraintsMatrixCoefsForX_.conservativeResize(nbElements + 1);
        generalConstraintsMatrixCoefsForY_.conservativeResize(nbElements + 1);
        generalConstraintsConstantPart_.conservativeResize(nbElements + 1);

        generalConstraintsMatrixCoefsForX_(nbElements) = p_[(i+1)%nbVertices](1) - p_[i](1);
        generalConstraintsMatrixCoefsForY_(nbElements) = p_[i](0) - p_[(i+1)%nbVertices](0);
        generalConstraintsConstantPart_(nbElements) =
            (p_[(i+1)%nbVertices](0) - p_[i](0))*p_[i](1) -
            (p_[(i+1)%nbVertices](1) - p_[i](1))*p_[i](0);
      }
    }
  }

  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(ConvexPolygon);
}
