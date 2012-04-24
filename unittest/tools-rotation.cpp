#include "../src/walkgen/tools.h"
#include "tools-rotation.h"
#include <iostream>

using namespace Eigen;


// The naive implementation" a brute force multiplication
MatrixXd MPCWalkgen::computeRMRt_Naive(const MatrixXd & chol, const MatrixXd & rot){
	assert(isDiagonalRotationMatrix(rot) && "The matrix rot is not 2.2 block diagonal");

	MatrixXd rotatedChol;
	rotatedChol.noalias() =rot*chol*rot.transpose();
	return rotatedChol;
}

//  This method considers only blocks of matrices 2.2, so as to reduce the
// number of multiplications
MatrixXd MPCWalkgen::computeRMRt_1a(const MatrixXd & chol, const MatrixXd & rot){
	assert(isDiagonalRotationMatrix(rot) && "The matrix rot is not 2.2 block diagonal");

	int N = chol.rows();
	MatrixXd rotatedChol( N, N );

	// first step: compute rot*chol
	for (int i=0; i<N/2; ++i)
	{
		const Eigen::Matrix2d & rot_i = rot.block<2,2>(2*i, 2*i);
		for (int j=0; j<N/2; ++j)
			rotatedChol.block<2,2>(2*i, 2*j).noalias() = rot_i*chol.block<2,2>(2*i, 2*j);
	}

	// second step: compute chol*col^T
	for (int j=0; j<N/2; ++j)
	{
		const Matrix2d & rot_j = (rot.block<2,2>(2*j, 2*j)).transpose();
		for (int i=0; i<N/2; ++i)
			rotatedChol.block<2,2>(2*i, 2*j) = rotatedChol.block<2,2>(2*i, 2*j)*rot_j;
	}
	return rotatedChol;
}

void MPCWalkgen::computeRMRt_1b(MatrixXd & mInOut, const MatrixXd & rot){
	assert(isDiagonalRotationMatrix(rot) && "The matrix rot is not 2.2 block diagonal");

	int N = mInOut.rows();

	// first step: compute rot*chol
	for (int i=0; i<N/2; ++i)
	{
		const Eigen::Matrix2d & rot_i = rot.block<2,2>(2*i, 2*i);
		for (int j=0; j<N/2; ++j)
			mInOut.block<2,2>(2*i, 2*j) = rot_i*mInOut.block<2,2>(2*i, 2*j);
	}

	// second step: compute chol*col^T
	for (int j=0; j<N/2; ++j)
	{
		const Matrix2d & rot_j = (rot.block<2,2>(2*j, 2*j)).transpose();
		for (int i=0; i<N/2; ++i)
			mInOut.block<2,2>(2*i, 2*j) = mInOut.block<2,2>(2*i, 2*j)*rot_j;
	}
}

//  This method considers only blocks of matrices 2.2, so as to reduce the
// number of multiplications. It is slightly different of the previous one
// (avoid the decomposition in two steps.
MatrixXd MPCWalkgen::computeRMRt_2a(const MatrixXd & chol, const MatrixXd & rot){
	assert(isDiagonalRotationMatrix(rot) && "The matrix rot is not 2.2 block diagonal");

	int N = chol.rows();
	MatrixXd rotatedChol( N, N );

	int n2 = N/2;
	for (int j=0; j<n2; ++j)
	{
		const Eigen::Matrix2d & rotT_j = rot.block<2,2>(2*j, 2*j).transpose();
		for (int i=0; i<n2; ++i)
		{
			rotatedChol.block<2,2>(2*i, 2*j).noalias() =
				rot.block<2,2>(2*i, 2*i) * chol.block<2,2>(2*i, 2*j)* rotT_j;
		}
	}
	return rotatedChol;
}


// --- Also consider the case where we multiply by this matrix
//  on the left
MatrixXd MPCWalkgen::compute_RM1(const MatrixXd & mIn, const MatrixXd & rot)
{
	assert(isDiagonalRotationMatrix(rot) && "The matrix rot is not 2.2 block diagonal");

	MatrixXd mOut(mIn);

	// first step: compute rot*chol
	for (int i=0; i<mIn.rows()/2; ++i)
	{
		const Eigen::Matrix2d & rot_i = rot.block<2,2>(2*i, 2*i);
		for (int j=0; j<mIn.cols()/2; ++j)
			mOut.block<2,2>(2*i, 2*j).noalias() = rot_i*mIn.block<2,2>(2*i, 2*j);
	}

	return mOut;
}

// --- Also consider the case where we multiply by this matrix
//  on the right
MatrixXd MPCWalkgen::compute_MRt1(const MatrixXd & mIn, const MatrixXd & rot)
{
	assert(isDiagonalRotationMatrix(rot) && "The matrix rot is not 2.2 block diagonal");
	MatrixXd mOut(mIn);

	// compute chol*col^T
	for (int j=0; j<mIn.cols()/2; ++j)
	{
		const Matrix2d & rot_j = (rot.block<2,2>(2*j, 2*j)).transpose();
		for (int i=0; i<mIn.rows()/2; ++i)
			mOut.block<2,2>(2*i, 2*j) = mIn.block<2,2>(2*i, 2*j)*rot_j;
	}
	return mOut;
}

