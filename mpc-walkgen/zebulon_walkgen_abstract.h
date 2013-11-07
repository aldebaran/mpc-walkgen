////////////////////////////////////////////////////////////////////////////////
///
///\file zebulon_walkgen_abstract.h
///\brief Main program for Zebulon
///\author Lafaye Jory
///\date 19/06/13
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

  class MPC_WALKGEN_NEW_API ZebulonWalkgenImpl
  {
    public:

      // For now weighting and config for Zebulon both exist in zebulon_walkgen_abstract
      // and shared_type to assure compatibility with other depo that currently use MPC-Walkgen
      // API like naoqi. Eventually shared_type will be deleted.
      class Weighting
      {
        public:
          Weighting();

          Scalar velocityTracking;
          Scalar positionTracking;
          Scalar copCentering;
          Scalar jerkMinimization;
      };

      class Config
      {
        public:
          Config();

          bool withCopConstraints;
          bool withComConstraints;
          bool withBaseMotionConstraints;
      };

      ZebulonWalkgenImpl();
      ~ZebulonWalkgenImpl();

      void setNbSamples(int nbSamples);
      void setSamplingPeriod(Scalar samplingPeriod);

      void setGravity(const Vector3& gravity);
      void setBaseCopHull(const std::vector<Vector3>& p);
      void setBaseComHull(const std::vector<Vector3>& p);
      void setComBodyHeight(Scalar comHeight);
      void setComBaseHeight(Scalar comHeight);
      void setBodyMass(Scalar mass);
      void setBaseMass(Scalar mass);
      void setWheelToBaseDistance(Scalar dist);
      void setAngleWheelToBaseCom(Scalar angle);

      void setVelRefInWorldFrame(const VectorX& velRef);
      void setPosRefInWorldFrame(const VectorX& posRef);
      void setCopRefInLocalFrame(const VectorX& copRef);
      void setComRefInLocalFrame(const VectorX& comRef);

      void setBaseVelLimit(Scalar limit);
      void setBaseAccLimit(Scalar limit);
      void setBaseJerkLimit(Scalar limit);

      void setBaseStateX(const VectorX& state);
      void setBaseStateY(const VectorX& state);
      void setBaseStateRoll(const VectorX& state);
      void setBaseStatePitch(const VectorX& state);
      void setComStateX(const VectorX& state);
      void setComStateY(const VectorX& state);

      void setWeightings(const MPCWalkgen::Weighting& weighting);
      void setConfig(const MPCWalkgen::Config& config);

      bool solve(Scalar feedBackPeriod);

      const VectorX& getBaseStateX() const;
      const VectorX& getBaseStateY() const;
      const VectorX& getComStateX() const;
      const VectorX& getComStateY() const;

    private:
      ZebulonWalkgen* walkgen;
  };
}

#endif //MPC_WALKGEN_ZEBULON_WALKGEN_ABSTRACT_H
