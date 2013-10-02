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



  ///QPMatrices
  void QPMatrices::normalizeMatrices()
  {
    Scalar objFactor = getNormalizationFactor(Q);
    Scalar ctrFactor = getNormalizationFactor(A);

    Q *= objFactor;
    p *= objFactor;

    A *= ctrFactor;
    bu *= ctrFactor;
    bl *= ctrFactor;
  }

  Scalar QPMatrices::getNormalizationFactor(const MatrixX& mat)
  {
    Scalar normalizationFactor = 1.0;
    for(int i=0; i<mat.rows(); ++i)
    {
      for(int j=0; j<mat.cols(); ++j)
      {
        Scalar v = std::abs(mat(i, j));
        if (v>EPSILON && v<normalizationFactor)
        {
          normalizationFactor = v;
        }
      }
    }

    return 1.0/normalizationFactor;
  }

  ///Convex Polygon
  ConvexPolygon::ConvexPolygon()
    :p_(0)
    ,xSupBound_(MAXIMUM_BOUND_VALUE)
    ,xInfBound_(-MAXIMUM_BOUND_VALUE)
    ,ySupBound_(MAXIMUM_BOUND_VALUE)
    ,yInfBound_(-MAXIMUM_BOUND_VALUE)
    ,generalConstraintsMatrixCoefsForX_()
    ,generalConstraintsMatrixCoefsForY_()
    ,generalConstraintsConstantPart_()
  {
    computeBoundsAndGeneralConstraintValues();
  }

  ConvexPolygon::ConvexPolygon(std::vector<Vector2> p)
    :p_(extractVertices(p))
    ,xSupBound_(MAXIMUM_BOUND_VALUE)
    ,xInfBound_(-MAXIMUM_BOUND_VALUE)
    ,ySupBound_(MAXIMUM_BOUND_VALUE)
    ,yInfBound_(-MAXIMUM_BOUND_VALUE)
    ,generalConstraintsMatrixCoefsForX_()
    ,generalConstraintsMatrixCoefsForY_()
    ,generalConstraintsConstantPart_()
  {
    computeBoundsAndGeneralConstraintValues();
  }

  ConvexPolygon::~ConvexPolygon(){}

  std::vector<Vector2> ConvexPolygon::extractVertices(
      const std::vector<Vector2>& points)
  {
    std::vector<Vector2> p;

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
                                           Vector2(points[index](0) - 1, points[index](1)),
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
      else if(std::abs(p[i](1)-p[lowestVerticesIndexes[0]](1)) < EPSILON)
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
      assert(lowestVerticesIndexes[0] < static_cast<int>(p.size()));
      return lowestVerticesIndexes[0];
    }
  }



  int ConvexPolygon::getIndexOfSmallestAngleVertice(int currIndex,
                                                    const Vector2& lastVertice,
                                                    const std::vector<Vector2>& ptot)
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

  void ConvexPolygon::computeBoundsAndGeneralConstraintValues()
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

 ///Interpolator
  Interpolator::Interpolator()
    :AinvNorm_(9,9)
    ,b_(9)
    ,abc_(9)
  {
    AinvNorm_<< 9./2. , 3./2.,  1./6. ,  9./2. ,  0.   , -1./12.,  9./2. , -3./2.,  1./6.  ,
               -9.    ,-3./2.,  5./12., -9.    ,  3./2.,  5./12., -9.    ,  9./2., -7./12. ,
                27./2., 3.   , -3./4. ,  27./2., -3./2., -1./2. ,  27./2., -6.   ,  3./4.  ,
               -9./2. ,-2.   ,  5./12., -9./2. ,  1./2.,  1./6. , -9./2. ,  2.   , -1./4.  ,
               -1./2. , 4./9., -7./108., 1./2. , -1./18.,-1./54.,  1./2. , -2./9.,  1./36. ,
                9./2. , 0.   , -1./12.,  9./2. , -3./2.,  1./6. ,  9./2. , -3.   ,  11./12.,
               -27./2., 0.   ,  1./4. , -27./2.,  9./2., -1./2. , -27./2.,  9.   , -9./4.  ,
                27./2., 0.   , -1./4. ,  27./2., -9./2.,  1./2. ,  27./2., -8.   ,  7./4.  ,
               -9./2. , 0.   ,  1./12., -9./2. ,  3./2., -1./6. , -7./2. ,  2.   , -5./12. ;
  }

  Interpolator::~Interpolator(){}

  void Interpolator::computePolynomialNormalisedFactors(VectorX &factor,
                                                        const Vector3 & initialstate,
                                                        const Vector3 & finalState,
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

    abc_=AinvNorm_*b_;

    factor(0) = abc_(0);
    factor.segment<8>(4) = abc_.segment<8>(1);
  }

  void Interpolator::selectFactors(Vector4 & subfactor,
                                   const VectorX & factor,
                                   Scalar t,
                                   Scalar T) const
  {
    if (t<=T/3.0)
    {
      subfactor = factor.segment<4>(0);
    }
    else if(t<=2.0*T/3.0)
    {
      subfactor = factor.segment<4>(4);
    }
    else
    {
      subfactor = factor.segment<4>(8);
    }
  }

}
