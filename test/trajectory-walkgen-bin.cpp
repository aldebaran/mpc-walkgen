////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/trajectory_walkgen.h>
#include <iostream>
#include <fstream>
#include <string>
#include <qi/os.hpp>
#include <iomanip>

using namespace MPCWalkgen;
typedef double Real;

void initLog(std::ofstream& out)
{
  out << "# Time pos vel acc jerk refpos refvel #" << std::endl;
}

void log(std::ofstream& out, const TrajectoryWalkgen<Real>& mpc,  Real t)
{
  out << t*1000. << " "
      << mpc.getState()(0) << " "
      << mpc.getState()(1) << " "
      << mpc.getState()(2) << " "
      << mpc.getJerk() << " "
      << 0 << " "
      << 0 << std::endl;
}

int main(int argc, char *argv[])
{
  if (argc!=2)
  {
    std::cout << "usage : ./trajectory-walkgen-bin "
              << "/home/jlafaye/devel/src/tools/motion-data/Python/experiences/motion/log"
              << std::endl;
  }



  std::ofstream out;
  out.open((std::string(argv[1])+"/Exp1000.txt").c_str());
  initLog(out);

  int nbSamples = 20;
  Real mpcSamplingPeriod = 0.1;
  Real mpcFeedbackPeriod = 0.02;

  TrajectoryWalkgen<Real> walkgen;
  walkgen.setNbSamples(nbSamples);
  walkgen.setSamplingPeriod(mpcSamplingPeriod);

  TrajectoryWalkgenWeighting<Real> weighting;
  weighting.velocityTracking = 1.0;
  weighting.positionTracking = 0.0;
  weighting.jerkMinimization = 0.000001;
  walkgen.setWeightings(weighting);

  TrajectoryWalkgenConfig<Real> config;
  config.withMotionConstraints = true;
  walkgen.setConfig(config);

  Type<Real>::VectorX velRef(nbSamples);
  velRef.fill(0.2);
  walkgen.setVelRefInWorldFrame(velRef);

  Type<Real>::VectorX posRef(nbSamples);
  posRef.fill(0.0);
  walkgen.setPosRefInWorldFrame(posRef);

  Type<Real>::VectorX state(3);
  state(0) = 0.0;
  state(1) = 0.0;
  state(2) = 0.0;
  walkgen.setState(state);

  walkgen.setVelLimit(1.0);
  walkgen.setAccLimit(1.0);
  walkgen.setJerkLimit(10.0);

  //Real jerk = 0.;
  for(Real t=0.0; t<5.0; t+=mpcFeedbackPeriod)
  {
    walkgen.solve(mpcFeedbackPeriod);

    log(out, walkgen, t);
  }

  out.close();

  return 0;
}
