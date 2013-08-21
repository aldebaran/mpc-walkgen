#include "type.h"

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

  ///Hull
  Hull::Hull()
    :p(3)
  {
    p[0] = Vector3(1.0, -1.0, 0.0);
    p[1] = Vector3(1.0, 1.0, 0.0);
    p[2] = Vector3(-1.0, 1.0, 0.0);

    computeBoundsVectors();
  }

  Hull::Hull(std::vector<Vector3> pp)
    :p(pp)
  {
    computeBoundsVectors();
  }

  void Hull::computeBoundsVectors()
  {

    for (size_t i=0; i<p.size(); ++i)
    {
      vectorMin_(0) = p[0](0);
      vectorMin_(1) = p[0](1);
      vectorMin_(2) = p[0](2);
      vectorMax_(0) = p[0](0);
      vectorMax_(1) = p[0](1);
      vectorMax_(2) = p[0](2);

      if(p[i](0)<vectorMin_(0))
      {
        vectorMin_(0) = p[i](0);
      }
      if(p[i](1)<vectorMin_(1))
      {
        vectorMin_(1) = p[i](1);
      }
      if(p[i](2)<vectorMin_(2))
      {
        vectorMin_(2) = p[i](2);
      }

      if(p[i](0)>vectorMax_(0))
      {
        vectorMax_(0) = p[i](0);
      }
      if(p[i](1)>vectorMax_(1))
      {
        vectorMax_(1) = p[i](1);
      }
      if(p[i](2)>vectorMax_(2))
      {
        vectorMax_(2) = p[i](2);
      }
    }
  }
}
