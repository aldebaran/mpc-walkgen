#ifndef PERTURBATION
#define PERTURBATION

////////////////////////////////////////////////////////////////////////////////
///
///\file	perturbation.h
///\brief	A class to apply perturbation on system
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author	Keith François
///\version	1.0
///\date	30/01/12
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/rigid-body-system.h>

namespace MPCWalkgen{

	class Perturbation{
	  public:
	    Perturbation(RigidBodySystem * robot);
	    ~Perturbation();

	    void applyForce(Axis axis, BodyType body, double f);
	    void applyAcc(Axis axis, BodyType body, double acc);
	    void applyVel(Axis axis, BodyType body, double vel);

	  private:
	    RigidBodySystem * robot_;

	};

}

#endif // PERTURBATION
