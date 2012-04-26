//  The purpose of this test is to find the fastest way to compute
// the rotation of the cholesky matrix, since this the rotation
// matrix is a sparse matrix

#include <cmath>
#include <cstdio>
#include <iostream>
#include <cstring>
#include <fstream>

#include "../src/common/tools.h"

#include "tools-rotation.h"

#include <Eigen/Core>

#include "../src/common/gettimeofday.h"
#ifdef WIN32
# include <Windows.h>
#endif //WIN32

using namespace Eigen;
using namespace MPCWalkgen;

MatrixXd buildRotationMatrix(int N, double angle){
	MatrixXd rot(MatrixXd::Zero(N, N));

	for( int i=0; i<N/2; ++i ){
		double cosYaw = cos(angle);
		double sinYaw = sin(angle);
		rot(2*i  ,2*i  ) =  cosYaw;
		rot(2*i+1,2*i  ) = -sinYaw;
		rot(2*i  ,2*i+1) =  sinYaw;
		rot(2*i+1,2*i+1) =  cosYaw;
	}
	return rot;
}

// A simple method to compare two matrices.
// a and b are the two matrics
// epsilon is the tolerance
bool compare(const MatrixXd & a, const MatrixXd & b, double epsilon)
{
	int numDiff =0;

	if (a.rows() != b.rows())
	{
		std::cerr << " Different number of rows " << std::endl;
		return false;
	}

	if (a.cols() != b.cols())
	{
		std::cerr << " Different number of columns " << std::endl;
		return false;
	}

	if ( !(a.isApprox(b,epsilon)) )
		for (int i = 0; i < a.rows(); ++i)
			for (int j = 0; j < a.cols(); ++j)
				if ( fabs(a(i, j) - b(i, j))>epsilon )
					++numDiff;

	return (numDiff == 0);
}

// Realize the test.
// Generates a random matrix of size n and a rotation matrix
// Then compares the results and the computation time.
// if the out file is provided and opened, writes in it the
// computation times
bool testCholeskyRotation(int n, std::ofstream &out)
{
  //Create a triangular matrix,          (a)
  //such as m(2*i,2*i) = m(2*i+1,2*i+1)  (b)
  //and  as m(2*i+i,2*j) = 0             (c)
  //and  as m(2*i  ,2*j+i) = 0           (d)
  MatrixXd chol (MatrixXd::Random(n,n));
  // (a)
  for (int i=0; i<n; ++i)
	for (int j=0; j<i; ++j)
	  chol(i,j) = 0;

  int n2 = static_cast <int> (floor(n/2.));
  // (b)
  for (int i=0; i<n2; ++i)
	  chol(2*i+1,2*i+1) = chol(2*i,2*i);

  // (c)
  for (int i=0; i<n2; ++i)
	  for (int j=0; j<n2; ++j)
		  chol(2*i+1,2*j) = 0;

  // (d)
  for (int i=0; i<n2; ++i)
	  for (int j=0; j<n2; ++j)
		  chol(2*i,2*j+1) = 0;

  MatrixXd rot=buildRotationMatrix(n,0.1);

  struct timeval tdeb, tend;

  gettimeofday(&tdeb,0);
  MatrixXd rotatedChol0 = computeRMRt_Naive    (chol.block(0,0,n,n), rot);
  gettimeofday(&tend,0);
  double timeM0= (1e6*tend.tv_sec + tend.tv_usec) -(1e6*tdeb.tv_sec + tdeb.tv_usec);

  // --
  gettimeofday(&tdeb,0);
  MatrixXd rotatedChol_1a = computeRMRt_1a(chol.block(0,0,n,n), rot);
  gettimeofday(&tend,0);
  double timeM1a= (1e6*tend.tv_sec + tend.tv_usec) -(1e6*tdeb.tv_sec + tdeb.tv_usec);

  gettimeofday(&tdeb,0);
  MatrixXd rotatedChol_1b = chol.block(0,0,n,n);
  computeRMRt_1b(rotatedChol_1b, rot);
  gettimeofday(&tend,0);
  double timeM1b= (1e6*tend.tv_sec + tend.tv_usec) -(1e6*tdeb.tv_sec + tdeb.tv_usec);

  // --
  gettimeofday(&tdeb,0);
  MatrixXd rotatedChol_2a = computeRMRt_2a(chol.block(0,0,n,n), rot);
  gettimeofday(&tend,0);
  double timeM2a= (1e6*tend.tv_sec + tend.tv_usec) -(1e6*tdeb.tv_sec + tdeb.tv_usec);

  gettimeofday(&tdeb,0);
  MatrixXd rotatedChol_2b = chol.block(0,0,n,n);
  rotateCholeskyMatrix(rotatedChol_2b, rot);
  gettimeofday(&tend,0);
  double timeM2b= (1e6*tend.tv_sec + tend.tv_usec) -(1e6*tdeb.tv_sec + tdeb.tv_usec);

  // --
  out << n << "\t\t " << timeM0;
  out <<      "\t\t " << timeM1a << "\t\t " << timeM1b;
  out <<      "\t\t " << timeM2a << "\t\t " << timeM2b << std::endl;

  bool result = true;
  result = compare (rotatedChol0, rotatedChol_1a, 1e-12);
  if (!result)
	  std::cerr << " Failed comparison for rotatedChol1_a " << std::endl;

  result = compare (rotatedChol0, rotatedChol_1b, 1e-12);
  if (!result)
	  std::cerr << " Failed comparison for rotatedChol_1b " << std::endl;

  result = compare (rotatedChol0, rotatedChol_2a, 1e-12);
  if (!result)
	  std::cerr << " Failed comparison for rotatedChol_2a " << std::endl;

  result = compare (rotatedChol0, rotatedChol_2b, 1e-12);
  if (!result)
	std::cerr << " Failed comparison for rotatedChol_2b " << std::endl;

  return result;
}






bool testRotation2(int n, int nbSP, std::ofstream &/*out*/)
{
	MatrixXd chol (MatrixXd::Random(2*(n+nbSP),2*(n+nbSP)));
	MatrixXd test0(chol);
	MatrixXd test1(chol);
	MatrixXd test2(chol);
	MatrixXd rot=buildRotationMatrix(2*n,0.1);

	struct timeval tdeb, tend;

	// First method tested: the brute force one.
	gettimeofday(&tdeb,0);
	test0.block(2*n,0,2*nbSP,2*n) =  test0.block(2*n,0,2*nbSP,2*n)*rot.transpose();
	test0.block(0,2*n,2*n,2*nbSP) =  rot*test0.block(0,2*n,2*n,2*nbSP);
	gettimeofday(&tend,0);
	double timeM0= (1e6*tend.tv_sec + tend.tv_usec) -(1e6*tdeb.tv_sec + tdeb.tv_usec);

	// Second method tested. The methods return matrices
	gettimeofday(&tdeb,0);
	test1.block(2*n,0,2*nbSP,2*n) =  compute_MRt1(test1.block(2*n, 0  , 2*nbSP, 2*n), rot);
	test1.block(0,2*n,2*n,2*nbSP) =  compute_RM1 (test1.block(0  , 2*n, 2*n, 2*nbSP), rot);
	gettimeofday(&tend,0);
	double timeM1= (1e6*tend.tv_sec + tend.tv_usec) -(1e6*tdeb.tv_sec + tdeb.tv_usec);


	// Third method tested. The methods modify the matrice given as argument
	gettimeofday(&tdeb,0);
	{
		MatrixXd block1 = test2.block(2*n, 0  , 2*nbSP, 2*n);
		computeMRt(block1, rot);
		test2.block(2*n, 0  , 2*nbSP, 2*n) = block1;
	}
	{
		MatrixXd block2 = test2.block(0  , 2*n, 2*n, 2*nbSP);
		computeRM (block2, rot);
		test2.block(0  , 2*n, 2*n, 2*nbSP) = block2;
	}
	gettimeofday(&tend,0);
	double timeM2= (1e6*tend.tv_sec + tend.tv_usec) -(1e6*tdeb.tv_sec + tdeb.tv_usec);

	// end of the tests, display the result.
	std::cout << "(" << n << ", " << nbSP << ")\t Init   " << timeM0
			<< "\tMeth1   " << timeM1  << "\tMeth2   " << timeM2 << std::endl;

	// Check that the results are the same for the three methods.
	bool result = true;
	if (! compare (test0, test1, 1e-12) )
	{
	  std::cerr << " Failed comparison between matrix 0 and 1 " << std::endl;
	  result = false;
	}

	if (! compare (test0, test2, 1e-12) )
	{
	  std::cerr << " Failed comparison between matrix 0 and 1 " << std::endl;
	  result = false;
	}
	return result;

}


int main ()
{
  std::ofstream out;
  std::cout << " Test of the multiplication R * TriangSup * Rt" << std::endl;
  out.open("time_test-rotation.dat");
  bool result = true;
  for (unsigned i=1; i<20; ++i)
	  result = testCholeskyRotation(2*i, out) && result;

  out.close();

  std::cout << " Test of the multiplications R*M and M*Rt" << std::endl;
  for (unsigned xi=4; xi<16; ++xi)
	  for (unsigned i=0; i<2; ++i)
		  testRotation2(2*xi, i, out);

  if (result == true)
    return 0;
  else
    return 1;
}
