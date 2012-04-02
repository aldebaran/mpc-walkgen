/*
*  The purpose of this test is to simply check the installation 
*  of lssol. Especially, we want to make sure that the link was succesful
*  The problem solved is hence quite simple:
*   min    || ( -1   -2) (x0)  - (-1) ||
*   x0,x1  || ( -2   -1) (x1)    ( 1) ||
*  The solution is (-1, 1)
*/

#include <cmath>
#include <cstdio>
#include <iostream>
#include <cstring>

#include <lssol/lssol.h>

int main ()
{
  int m=2; 
  int n=2; 
  int nclin=0; 
  int ldC=n; 
  int ldA=m; 
  double* C = new double [n]; 
  C[0]=0;
  C[1]=0;
  double* bl = new double [n];
  bl[0]=-10;
  bl[1]=-10;
  double* bu = new double [n];
  bu[0]=10;
  bu[1]=10;  
  double* cvec = new double [n];
  int* istate = new int [n+nclin]; 
  int* kx = new int[n];
  double* x = new double [n];
  x[0]=0;
  x[1]=0;
  double* A = new double [m*n];
  A[0]=-1;A[1]=-2;
  A[2]=-2;A[3]=-1;
  double* b = new double [m];
  b[0]=-1;
  b[1]=1;
  int inform;
  int iter; 
  double obj; 
  double* clamda = new double [n+nclin]; 
  int* iw = new int[n+100]; 
  int leniw=n+100; 
  double* w = new double[n+100]; 
  int lenw=10*n+100;

  sendOption("Print Level = 100"); 
  
  lssol_(&m, &n, 
    &nclin, &ldC, &ldA, 
    C, bl, bu, cvec, 
    istate, kx, x, A, b, 
    &inform, &iter, &obj, clamda, 
    iw, &leniw, w, &lenw);

  bool result = ((fabs(x[0] + 1) < 1e-9)  && (fabs(x[1] - 1) < 1e-9));

  delete[] w;
  delete[] iw;
  delete[] clamda;
  delete[] b;
  delete[] A;
  delete[] x;
  delete[] kx;
  delete[] istate;
  delete[] cvec;
  delete[] bu;
  delete[] bl;
  delete[] C;

  if (result == true)
    return 0;
  else
    return 1;
}
