#pragma once
#ifndef MPC_WALKGEN_INTERPOLATION_H
#define MPC_WALKGEN_INTERPOLATION_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	interpolation.h
///\brief	A tools class wich provide some interpolation methods
///\author	Lafaye Jory
///\author      Keith François
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////



#include "types.h"
#include "tools.h"
#include <Eigen/Dense>

namespace MPCWalkgen{

  class Interpolation{

    public:
      Interpolation();
      ~Interpolation();

      void computeInterpolationByJerk(Eigen::VectorXd & solutionX, Eigen::VectorXd & solutionY,
                                      const BodyState & state,const LinearDynamics & dyn,
                                      double jerkX, double jerkY) const;

      void computeInterpolationByJerk(Eigen::VectorXd & solution, const Eigen::VectorXd & state,
                                      const LinearDynamics & dyn, double jerk) const;
      /// \brief Computes normalised spline factors for each of its three polynoms (of degree three).
      ///Constraints are at both ends of the spline, and at the two junction between
      ///the three polynoms
      void computePolynomialNormalisedFactors(Eigen::VectorXd &factor,
                                              const Eigen::Vector3d & initialstate,
                                              const Eigen::Vector3d & finalState, double T ) const;

      /// \brief select the proper 4 factors corresponding to one of the three polynoms in a
      ///normalised spline of degree three
      void selectFactors(Eigen::Vector4d & subfactor,
                         const Eigen::VectorXd & factor,
                         double t, double T) const;

    private:
      /// \brief We have a cubic spline of three polynoms with 3 constraints (position, velocity
      ///and acceleration) on both edges of the spline, and 3 constraints (position velocity and
      ///acceleration) for each junctions between the three polynoms.
      ///The spline is normalised, and the junction are set at x=1/3 and x=2/3.
      ///We have then 2*3 + 2*3 = 12 equations (one per constraint), and 12 factor
      ///(4 for each polynom) to compute.
      ///As we have 3 constraints at x=0 for the first polynom, one can easily compute its three
      ///first factors. The 9 left factors can be obtained from the 9 equations corresponding to the
      ///9 left constraints.
      ///This set of 9 linear equations can be written as a matricial equation AX=b. AinvNorm_ is
      ///then the inverse of matrix A, and b_ is the matrix b
      Eigen::MatrixXd AinvNorm_;
      mutable Eigen::VectorXd b_;
      mutable Eigen::VectorXd abc_;
  };
}




#endif // MPC_WALKGEN_INTERPOLATION_H
