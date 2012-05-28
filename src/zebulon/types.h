#pragma once
#ifndef MPC_WALKGEN_ZEBULON_TYPES_H
#define MPC_WALKGEN_ZEBULON_TYPES_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	types.h
///\brief	Definition of types used in MPC
///\author	Lafaye Jory
///\version	1.0
///\date	02/05/12
///
////////////////////////////////////////////////////////////////////////////////


#include <mpc-walkgen/zebulon/sharedpgtypes.h>
#include "../common/types.h"
namespace MPCWalkgen{
  namespace Zebulon{

    struct GlobalSolution{

      GlobalSolution();

      MPCSolution mpcSolution;

      // attributes
      Eigen::VectorXd qpSolution;
      Eigen::VectorXd initialSolution;

      Eigen::VectorXi constraints;
      Eigen::VectorXi initialConstraints;

      Eigen::VectorXd qpSolutionOrientation;
      Eigen::VectorXd initialSolutionOrientation;

      Eigen::VectorXi constraintsOrientation;
      Eigen::VectorXi initialConstraintsOrientation;

      bool useWarmStart;

    };
  }
}

#endif // MPC_WALKGEN_ZEBULON_TYPES_H
