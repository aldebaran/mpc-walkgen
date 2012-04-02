#ifndef TOOLS_ROTATION
#define TOOLS_ROTATION

// This file contains several implementations of the multiplications
// R.M, M.R^T, R.M.R^T
#include <Eigen/Core>


namespace MPCWalkgen{

	// The naive implementation" a brute force multiplication
	Eigen::MatrixXd computeRMRt_Naive(const Eigen::MatrixXd & chol, const Eigen::MatrixXd & rot);

	//  This method considers only blocks of matrices 2.2, so as to reduce the
	// number of multiplications
	Eigen::MatrixXd computeRMRt_1a(const Eigen::MatrixXd & chol, const Eigen::MatrixXd & rot);
	void     computeRMRt_1b(Eigen::MatrixXd & mInOut, const Eigen::MatrixXd & rot);

	//  This method considers only blocks of matrices 2.2, so as to reduce the
	// number of multiplications. It is slightly different of the previous one
	// (avoid the decomposition in two steps.
	Eigen::MatrixXd computeRMRt_2a(const Eigen::MatrixXd & chol, const Eigen::MatrixXd & rot);




	// --- Also consider the case where we multiply by this matrix
	//  on the left
	Eigen::MatrixXd compute_RM1(const Eigen::MatrixXd & mIn, const Eigen::MatrixXd & rot);

	// --- Also consider the case where we multiply by this matrix
	//  on the right
	Eigen::MatrixXd compute_MRt1(const Eigen::MatrixXd & mIn, const Eigen::MatrixXd & rot);
}

#endif //TOOLS_ROTATION
