////////////////////////////////////////////////////////////////////////////////
///
///\file	zebulon_walkgen_abstract.h
///\brief	Main program for Zebulon
///\author Lafaye Jory
///\version	1.0
///\date	19/06/13
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_ZEBULON_WALKGEN_ABSTRACT_H
#define MPC_WALKGEN_ZEBULON_WALKGEN_ABSTRACT_H

#include <mpc-walkgen/shared_type.h>
#include <vector>

namespace MPCWalkgen
{
  class ZebulonWalkgen;

  class MPC_WALKGEN_NEW_API ZebulonWalkgenImp
  {
  public:
    ZebulonWalkgenImp();
    ~ZebulonWalkgenImp();

    void setNbSamples(int nbSamples);
    void setSamplingPeriod(Scalar samplingPeriod);

    void setGravity(const Vector3& gravity);
    void setBaseHull(const std::vector<Vector3> p);
    void setComHeight(Scalar comHeight);

    void setVelRefInWorldFrame(const VectorX& velRef);
    void setPosRefInWorldFrame(const VectorX& posRef);
    void setCopRefInLocalFrame(const VectorX& copRef);

    void setBaseVelLimit(Scalar limit);
    void setBaseAccLimit(Scalar limit);
    void setBaseJerkLimit(Scalar limit);

    void setBaseStateX(const VectorX& state);
    void setBaseStateY(const VectorX& state);
    void setComStateX(const VectorX& state);
    void setComStateY(const VectorX& state);

    void setWeightings(const Weighting& weighting);
    void setConfig(const Config& config);

    void solve(Scalar feedBackPeriod);

    const VectorX& getBaseStateX();
    const VectorX& getBaseStateY();
    const VectorX& getComStateX();
    const VectorX& getComStateY();

  private:
    ZebulonWalkgen* walkgen;
  };
}

#endif //MPC_WALKGEN_ZEBULON_WALKGEN_ABSTRACT_H
