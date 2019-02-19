#include <mpc-walkgen/zebulon_walkgen.h>
#include <iostream>
#include <qi/clock.hpp>
#include <iomanip>

using namespace MPCWalkgen;
typedef double Real;

void updatePsi(ZebulonWalkgen<Real>& walkgen, Type<Real>::VectorX& psi)
{
  Real b = walkgen.getBaseStateY()(0);
  Real c = walkgen.getComStateY()(0);
  Real ddb = walkgen.getBaseStateY()(2);
  Real ddc = walkgen.getComStateY()(2);

  Real g = 9.81;
  Real m = 13.5;
  Real M = 16.5;
  Real h = 0.73;
  Real L = 0.13;

  Real T = 0.02;

  psi(0) += psi(1)*T+psi(2)*T*T/2;
  psi(1) += psi(2)*T;
  psi(2) = (g*(m*h+M*L)*psi(0)-m*g*(c-b)+M*L*ddb+m*h*ddc+M*g*0.24)/(m*h*h+M*L*L);
;

  if (psi(0)>-0.01)
  {
    psi(0) = 0.0;
  }
  static bool touch = false;
  if (psi(0)>=0.0 || touch){
    psi.fill(0.0);
    touch = true;
    ZebulonWalkgenWeighting<Real> weighting;
    weighting.copCentering = 10.0;
    weighting.comCentering = 100.0;
    weighting.velocityTracking = 10.0;
    weighting.positionTracking = 0.0;
    weighting.jerkMinimization = 0.00001;
    weighting.tiltMinimization = 0.0;
    weighting.tiltVelMinimization = 0.0;
    walkgen.setWeightings(weighting);
    ZebulonWalkgenConfig<Real> config;
    config.withBaseMotionConstraints = false;
    config.withComConstraints = false;
    config.withCopConstraints = true;
    walkgen.setConfig(config);
  }

}

int main(void)
{

  int nbSamples = 10;

  ZebulonWalkgen<Real> walkgen;

  walkgen.setNbSamples(nbSamples);
  walkgen.setSamplingPeriod(0.2);
  walkgen.setComBodyHeight(0.73);
  walkgen.setComBaseHeight(0.13);
  walkgen.setBodyMass(13.5);
  walkgen.setBaseMass(16.5);
  walkgen.setTiltContactPointOnTheGroundInLocalFrameX(0.0);
  walkgen.setTiltContactPointOnTheGroundInLocalFrameY(0.15);

  Real copLimitMin = 0.1;
  Real copLimitMax = 0.15;
  Type<Real>::vectorOfVector3 p(8);
  p[0] = Type<Real>::Vector3(copLimitMin, copLimitMin, 0.0);
  p[1] = Type<Real>::Vector3(0.0, copLimitMax, 0.0);
  p[2] = Type<Real>::Vector3(-copLimitMin, copLimitMin, 0.0);
  p[3] = Type<Real>::Vector3(-copLimitMax, 0.0, 0.0);
  p[4] = Type<Real>::Vector3(-copLimitMin, -copLimitMin, 0.0);
  p[5] = Type<Real>::Vector3(0.0, -copLimitMax, 0.0);
  p[6] = Type<Real>::Vector3(copLimitMin, -copLimitMin, 0.0);
  p[7] = Type<Real>::Vector3(copLimitMax, 0.0, 0.0);
  walkgen.setBaseCopHull(p);
  walkgen.setBaseComHull(p);

  ZebulonWalkgenWeighting<Real> weighting;
  weighting.copCentering = 10.0;
  weighting.comCentering = 100.0;
  weighting.velocityTracking = 100.0;
  weighting.positionTracking = 0.0;
  weighting.jerkMinimization = 0.00001;
  weighting.tiltMinimization = 0.0;
  weighting.tiltVelMinimization = 0.0;
/*
  weighting.copCentering = 0.0;
  weighting.comCentering = 0.1;
  weighting.velocityTracking = 0.1;
  weighting.positionTracking = 0.0;
  weighting.jerkMinimization = 0.001;
  weighting.tiltMinimization = 10.0;
*/
  walkgen.setWeightings(weighting);

  ZebulonWalkgenConfig<Real> config;
  config.withBaseMotionConstraints = false;
  config.withComConstraints = false;
  config.withCopConstraints = true;
  walkgen.setConfig(config);

  Type<Real>::VectorX velRef(2*nbSamples);
  velRef.segment(0, nbSamples).fill(0.0);
  velRef.segment(nbSamples, nbSamples).fill(0.0);
  walkgen.setVelRefInWorldFrame(velRef);

  Type<Real>::VectorX posRef(2*nbSamples);
  posRef.fill(0.0);
  walkgen.setPosRefInWorldFrame(posRef);

  Type<Real>::VectorX copRef(2*nbSamples);
  copRef.segment(0, nbSamples).fill(0.0);
  copRef.segment(nbSamples, nbSamples).fill(0.0);
  walkgen.setCopRefInLocalFrame(copRef);

  Type<Real>::VectorX baseState(3);
  baseState(0) = 0.0;
  baseState(1) = 0.0;
  baseState(2) = 0.0;
  walkgen.setBaseStateX(baseState);

  walkgen.setBaseVelLimit(3.0);
  walkgen.setBaseAccLimit(4.0);
  walkgen.setBaseJerkLimit(160.0);

  Real samplingFeedback = 0.02;

  baseState(0) = 0.0;
  baseState(1) = 0.0;
  baseState(2) = 0.0;

    std::cout << std::setw(12) << std::setprecision(4) << "time"
              << "     ||"
              << std::setw(12) << std::setprecision(4) << "angle"
              << std::setw(12) << std::setprecision(4) << "vel angle"
              << std::setw(12) << std::setprecision(4) << "acc angle"
              << "     ||"
              << std::setw(12) << std::setprecision(4) << "acc base"
              << std::setw(12) << std::setprecision(4) << "acc com"
              << "     ||"
              << std::setw(12) << std::setprecision(4) << "vel base"
              << std::setw(12) << std::setprecision(4) << "vel com"
              << "     ||"
              << std::setw(12) << std::setprecision(4) << "pos base"
              << std::setw(12) << std::setprecision(4) <<"pos com"
              << "     ||"
              << std::setw(12) << std::setprecision(4) << "delta base com"
              << std::endl
              << "_________________________________________________________"
              << "_________________________________________________________"
              << "_________________________________________________________"
              << std::endl;
  for(Real t=4.0f; t<7.4f; t+=samplingFeedback)
  {


    Real acc = -5.5;
    if (t>=5.0f && t<5.022f){
      std::cout << "!!!!!!!!!!!!!!!!!! TILT !!!!!!!!!!!!!!!!" << std::endl;


      baseState(0) = acc*0.02*0.02*6*6;
      baseState(1) = acc*0.02*6;
      baseState(2) = acc;

    }
    if (t>=5.02f && t<5.22f){
      baseState(0) += acc*0.02*0.02;
      baseState(1) += acc*0.02;
      baseState(2) = acc;
    }
    if (t>=5.22f && t<5.24f){
      std::cout << "!!!!!!!!!!!!!!!!!! end TILT !!!!!!!!!!!!!!!!" << std::endl;
      ZebulonWalkgenWeighting<Real> weighting;
      weighting.copCentering = 0.0;
      weighting.comCentering = 100.0;
      weighting.velocityTracking = 0.001;
      weighting.positionTracking = 0.0;
      weighting.jerkMinimization = 0.001;
      weighting.tiltMinimization = 10.0;
      weighting.tiltVelMinimization = 10.0;
      walkgen.setWeightings(weighting);
      ZebulonWalkgenConfig<Real> config;
      config.withBaseMotionConstraints = false;
      config.withComConstraints = false;
      config.withCopConstraints = false;
      walkgen.setConfig(config);
    }


    walkgen.setBaseStatePitch(baseState);

    bool s = walkgen.solve(samplingFeedback);

    if (t<5.0f){
        baseState.fill(0.0);
    }else{
      if (t>=5.24){
        updatePsi(walkgen, baseState);
      }
    }
    std::cout << std::setw(12) << std::setprecision(3) << t
              << "     ||"
              << std::setw(12) << std::setprecision(3) << baseState(0)
              << std::setw(12) << std::setprecision(3) << baseState(1)
              << std::setw(12) << std::setprecision(3) << baseState(2)
              << "     ||"
              << std::setw(12) << std::setprecision(3) << walkgen.getBaseStateY()(2)
              << std::setw(12) << std::setprecision(3) << walkgen.getComStateY()(2)
              << "     ||"
              << std::setw(12) << std::setprecision(3) << walkgen.getBaseStateY()(1)
              << std::setw(12) << std::setprecision(3) << walkgen.getComStateY()(1)
              << "     ||"
              << std::setw(12) << std::setprecision(3) << walkgen.getBaseStateY()(0)
              << std::setw(12) << std::setprecision(3) << walkgen.getComStateY()(0)
              << "     ||"
              << std::setw(12) << std::setprecision(3) << walkgen.getComStateY()(0)
                                                       -  walkgen.getBaseStateY()(0)
              << std::endl;
    if (!s)
    {
      break;
    }

    //std::cout << "Solving time : " << qi::to_string(total) << std::endl;
  }

  return 0;
}
