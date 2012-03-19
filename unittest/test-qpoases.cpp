
/*
*  The purpose of this test is to simply check the installation
*  of qpoases. Especially, we want to make sure that the link was succesful
*  The problem solved is hence quite simple:
*   min    || ( -1   -2) (x0)  - (-1) ||
*   x0,x1  || ( -2   -1) (x1)    ( 1) ||
*  The solution is (-1, 1)
*/

#include <cmath>
#include <cstdio>
#include <iostream>
#include <cstring>

#ifdef USE_QPOASES_3_0
# include <qpOASES/QProblem.hpp>
 typedef double Real;
#else //
# include <QProblem.hpp>
#endif //USE_QPOASES_3_0

using namespace qpOASES;
int main ()
{

  int n=2;
  int itt=100;
  Real bl[n];
  bl[0]=-10;
  bl[1]=-10;
  Real bu[n];
  bu[0]=10;
  bu[1]=10;
  Real x[n];
  x[0]=0;
  x[1]=0;
  Real H[n*n];
  H[0]=5;H[1]=4;
  H[2]=4;H[3]=5;
  Real p[n];
  p[0]=1;
  p[1]=-1;

  QProblem qp(n, 0);

  qp.setPrintLevel(PL_HIGH);

  qp.init(H, p, (Real*)0,
            bl,bu,
          (Real*) 0,(Real*)0,
           itt, (Real*)0);

  qp.getPrimalSolution(x);


  bool result = ((fabs(x[0] + 1) < 1e-9)  && (fabs(x[1] - 1) < 1e-9));

  std::cout << "solution : " << x[0] << " " << x[1] << std::endl;

  if (result == true)
    return 0;
  else
    return 1;
}
