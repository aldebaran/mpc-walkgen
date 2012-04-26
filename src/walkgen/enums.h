#ifndef PGENUM
#define PGENUM

////////////////////////////////////////////////////////////////////////////////
///
///\file	types.h
///\brief	Definition of enums used in MPC
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author	Keith François
///\version	1.0
///\date	05/01/12
///
////////////////////////////////////////////////////////////////////////////////

namespace MPCWalkgen{

	// TODO: Redundant
	// Every body requires adding of several enums of this type
	// With three bodies the number of enums of this type alone will be 8*3+1=25 
	// 5 bodies: 8*5+1=41
	enum DynamicMatrixType{
		posDynamic,
		velDynamic,
		accDynamic,
		jerkDynamic,
		copDynamic,
		interpolationPos,
		interpolationVel,
		interpolationAcc,
		interpolationCoP
	};


	enum HullType{
		FootHull,
		CoPHull
	};


}

#endif //PGENUM
