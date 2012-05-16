#ifndef QP_GENERATOR_ZEBULON
#define QP_GENERATOR_ZEBULON

////////////////////////////////////////////////////////////////////////////////
///
///\file	qp-generator.h
///\brief	A class to compute QP elements (objective, constraints, warmstart)
///\author	Lafaye Jory
///\version	1.0
///\date	02/05/12
///
////////////////////////////////////////////////////////////////////////////////



#include "types.h"

#include "../common/qp-solver.h"
#include "rigid-body-system.h"

#include <Eigen/Dense>
#include <vector>

namespace MPCWalkgen{
    namespace Zebulon{
    class QPGenerator{

    public:
      QPGenerator(QPSolver * solver, QPSolver * solverOrientation,
                  VelReference * velRef, QPPonderation * ponderation,
                  RigidBodySystem * robot, const MPCData * generalData);
      ~QPGenerator();

      void precomputeObjective();

      void precomputeObjectiveAndConstraintsOrientation();

      void buildObjective();

      void buildConstraints();
      void buildConstraintsCoP();
      void buildConstraintsCoM();
      void buildConstraintsBaseVelocity();
      void buildConstraintsBaseAcceleration();
      void buildConstraintsBaseJerk();

      void computeWarmStart(MPCSolution & result);

      void buildObjectiveOrientation();

      void buildConstraintsOrientation();
      void buildConstraintsBaseVelocityOrientation();
      void buildConstraintsBaseAccelerationOrientation();
      void buildConstraintsBaseJerkOrientation();

      void computeWarmStartOrientation(MPCSolution & result);

      void computeOrientationReferenceVector();

      void computeReferenceVector(const MPCSolution & result);

    private:

      void computeOrientationMatrices(const MPCSolution & result);

    private:

      QPSolver * solver_;
      QPSolver * solverOrientation_;
      RigidBodySystem * robot_;
      VelReference * velRef_;
      QPPonderation * ponderation_;
      const MPCData * generalData_;

      Eigen::VectorXd tmpVec_;
      Eigen::VectorXd tmpVec2_;
      Eigen::MatrixXd tmpMat_;
      Eigen::MatrixXd tmpMat2_;

      std::vector<Eigen::MatrixXd> Qconst_;
      std::vector<Eigen::MatrixXd> pconstCoMX_;
      std::vector<Eigen::MatrixXd> pconstCoMB_;
      std::vector<Eigen::MatrixXd> pconstBaseX_;
      std::vector<Eigen::MatrixXd> pconstBaseB_;
      std::vector<Eigen::MatrixXd> pconstRef_;


      std::vector<Eigen::MatrixXd> QconstOr_;
      std::vector<Eigen::MatrixXd> pconstOrCoMYaw_;
      std::vector<Eigen::MatrixXd> pconstOrRef_;

      Eigen::VectorXd yaw_;
      Eigen::VectorXd cosYaw_;
      Eigen::VectorXd sinYaw_;
      Eigen::MatrixXd Rxx_;
      Eigen::MatrixXd Rxy_;
      Eigen::MatrixXd Ryx_;
      Eigen::MatrixXd Ryy_;

    };
  }
}

/*! \fn MPCWalkgen::QPGenerator::QPGenerator(QPPreview * preview, QPSolver * solver,
*					VelReference * velRef, QPPonderation * ponderation,
*					RigidBodySystem * robot, const MPCData * generalData)
* \brief Constructor
*/

/*! \fn MPCWalkgen::QPGenerator::buildObjective(const MPCSolution & result)
* \brief Build matrix Q and vector p for the QP
*/

/*! \fn MPCWalkgen::QPGenerator::buildConstraints(const MPCSolution & result)
* \brief build matrix A and vectors BU, BL, XU, XL for the QP
*/

/*! \fn MPCWalkgen::QPGenerator::computeWarmStart(MPCSolution & result)
* \brief Compute a feasible solution and an active-set of previewed constraints
*/


/*! \fn MPCWalkgen::QPGenerator::computeReferenceVector(const MPCSolution & result)
* \brief Compute the reference vector (constant over the horizon)
*/


#endif //QP_GENERATOR_ZEBULON
