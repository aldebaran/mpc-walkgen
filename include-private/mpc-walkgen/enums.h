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

	//TODO: The QP matrices and vectors in the class solver can be encapsulated 
	// QPProblem & QP = solver_.QP();
	// QP.vectoP.addTerm();
	// instead of solver_->matrix(VECTOR_P).addTerm();
	// which would also make these enums unnecessary
	enum QPMatrixType{
		matrixQ,
		matrixA,
		vectorP,
		vectorBU,
		vectorBL,
		vectorXU,
		vectorXL
	};


	enum HullType{
		FootHull,
		CoPHull
	};


	enum Axis{
		X,
		Y,
		Z,
		Yaw
	};

}

#endif //PGENUM
