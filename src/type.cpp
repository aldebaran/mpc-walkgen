#include "type.h"

#include <Eigen/Geometry>
#define _USE_MATH_DEFINES
namespace MPCWalkgen
{

  ///LinearDynamic
  void LinearDynamic::reset(int nbSamples,
                            int stateVectorSize,
                            int variableVectorSize)
  {
    U.setZero(nbSamples, variableVectorSize);
    UT.setZero(variableVectorSize, nbSamples);

    if(nbSamples==variableVectorSize)
    {
      Uinv.setZero(nbSamples, variableVectorSize);
      UTinv.setZero(variableVectorSize, nbSamples);
    }
    else
    {
      Uinv.setConstant(nbSamples,
                       variableVectorSize,
                       std::numeric_limits<Scalar>::quiet_NaN());
      UTinv.setConstant(variableVectorSize,
                        nbSamples,
                        std::numeric_limits<Scalar>::quiet_NaN());
    }
    S.setZero(nbSamples, stateVectorSize);
    ST.setZero(stateVectorSize, nbSamples);
  }

  ///Convex Polygon
  ConvexPolygon::ConvexPolygon()
    :p_(0)
    ,xSupBound_(std::numeric_limits<Scalar>::max())
    ,xInfBound_(-std::numeric_limits<Scalar>::max())
    ,ySupBound_(std::numeric_limits<Scalar>::max())
    ,yInfBound_(-std::numeric_limits<Scalar>::max())
  {
    generalConstraintsMatrixCoefsForX_.setZero(1);
    generalConstraintsMatrixCoefsForY_.setZero(1);
    generalConstraintsConstantPart_.setConstant(1, std::numeric_limits<Scalar>::max());

    xComputeBoundsAndGeneralConstraintValues();
  }

  ConvexPolygon::ConvexPolygon(std::vector<Vector2> p)
    :p_(p)
    ,xSupBound_(std::numeric_limits<Scalar>::max())
    ,xInfBound_(-std::numeric_limits<Scalar>::max())
    ,ySupBound_(std::numeric_limits<Scalar>::max())
    ,yInfBound_(-std::numeric_limits<Scalar>::max())
  {
    generalConstraintsMatrixCoefsForX_.setZero(1);
    generalConstraintsMatrixCoefsForY_.setZero(1);
    generalConstraintsConstantPart_.setConstant(1, std::numeric_limits<Scalar>::max());

    p_ = extractVertices(p_);

    xComputeBoundsAndGeneralConstraintValues();
  }

  ConvexPolygon::~ConvexPolygon(){}

  std::vector<Vector2> ConvexPolygon::extractVertices(
      const std::vector<Vector2>& points)
  {
    // Adding the first vertex
    int index = getIndexOfLowestAndLeftmostVertice(points);
    std::vector<Vector2> pout;
    pout.reserve(points.size());
    pout.push_back(points[index]);

    // Finding the index of the second vertex
    index = getIndexOfSmallestAngleVertice(index,
                                           Vector2(points[index](0) - 1, points[index](1)),
                                           points);

    // Adding the other vertices until we reach the first index, which
    // means the convex polygon has been completed
    while(index!=getIndexOfLowestAndLeftmostVertice(points))
    {
      pout.push_back(points[index]);
      index = getIndexOfSmallestAngleVertice(index, *(pout.end()-2), points);
    }

    return pout;
  }


  Scalar ConvexPolygon::angleBetweenVecs(const Vector2& vec1, const Vector2& vec2)
  {
    if(vec1.isZero() ||
       vec2.isZero() ||
       vec1.isApprox(vec2, EPSILON))
    {
      return 0.0;
    }
    else if (vec1.isApprox(-vec2, EPSILON))
    {
      return M_PI;
    }
    else
    {
      Scalar angle = vec1.dot(vec2)/(vec1.norm()*vec2.norm());

      if(vec1(0)*vec2(1) - vec1(1)*vec2(0) >= 0.0)
      {
        return std::acos(angle);
      }
      else
      {
        return -std::acos(angle);
      }
    }
  }


  int ConvexPolygon::getIndexOfLowestAndLeftmostVertice(const std::vector<Vector2>& p)
  {
    //Storing indexes of the vertices with the lowest value of Y coordinate
    std::vector<int> lowestVerticesIndexes;
    lowestVerticesIndexes.push_back(0);

    for (size_t i=1; i<p.size(); ++i)
    {
      if(p[i](1)<p[lowestVerticesIndexes[0]](1))
      {
        lowestVerticesIndexes.erase(lowestVerticesIndexes.begin(),
                                    lowestVerticesIndexes.end());
        lowestVerticesIndexes.push_back(i);
      }
      else if(std::abs(p[i](1)==p[lowestVerticesIndexes[0]](1)) < EPSILON)
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
      return index;
    }
    else
    {
      return lowestVerticesIndexes[0];
    }
  }



  int ConvexPolygon::getIndexOfSmallestAngleVertice(unsigned int currIndex,
                                                    const Vector2& lastVertice,
                                                    const std::vector<Vector2>& ptot)
  {
    //Creating vector of indexes of the candidate for the next vertex
    std::vector<int> indexesTab;

    //This if-else statement prevent from pushing back index 0 if currIndex=0
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
      //We do not check the current point
      if(!ptot[i].isApprox(ptot[currIndex], EPSILON))
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
                                          ptot[indexesTab[0]] - ptot[currIndex])) < EPSILON)
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

  void ConvexPolygon::xComputeBoundsAndGeneralConstraintValues()
  {
    // We check whether or not the ith convex polygon side is parallel to axis X or Y.
    // If so, it is stored in bounds constraints vectors. If not, it is a general constraint.
    // One should remember that vertices are ordered counter-clockwise.
    // Note that if two vertices are closer in norm than EPSILON, one of them is discarded
    // in extractVertices(). The constructor then ensures that such a case cannot happen
    // in this function.
    for (size_t i=0; i<p_.size(); ++i)
    {
      Vector2 hullEdge = p_[(i+1)%p_.size()] - p_[i];

      if(std::abs(hullEdge(0))<EPSILON)
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
      else if(std::abs(hullEdge(1))<EPSILON)
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
        unsigned int nbElements = generalConstraintsMatrixCoefsForX_.rows();
        //TODO: template class to set _MaxRows for these 3 VectorX and to
        //avoid reallocations
        generalConstraintsMatrixCoefsForX_.conservativeResize(nbElements + 1);
        generalConstraintsMatrixCoefsForY_.conservativeResize(nbElements + 1);
        generalConstraintsConstantPart_.conservativeResize(nbElements + 1);

        generalConstraintsMatrixCoefsForX_(nbElements) = p_[(i+1)%p_.size()](1) - p_[i](1);
        generalConstraintsMatrixCoefsForY_(nbElements) = p_[i](0) - p_[(i+1)%p_.size()](0);
        generalConstraintsConstantPart_(nbElements) = (p_[(i+1)%p_.size()](1) - p_[i](1))*p_[i](0) -
            (p_[i](0) - p_[(i+1)%p_.size()](0))*p_[i](1);
      }
    }
  }

}
