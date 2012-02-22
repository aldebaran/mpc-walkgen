#include <mpc-walkgen/types.h>


namespace MPCWalkgen
{

ConvexHull::ConvexHull()
:x(0),y(0),z(0)
,A(0),B(0),C(0),D(0) {
}

// TODO: Necessary?
ConvexHull & ConvexHull::operator=(const ConvexHull & hull){
	x=hull.x;
	y=hull.y;
	z=hull.z;
	A=hull.A;
	B=hull.B;
	C=hull.C;
	D=hull.D;

	return *this;
}

void ConvexHull::resize(int size){
	if (size != x.rows()){
		x.setZero(size);
		y.setZero(size);
		A.setZero(size);
		B.setZero(size);
		C.setZero(size);
		D.setZero(size);
	}else{
		x.fill(0);
		y.fill(0);
		A.fill(0);
		B.fill(0);
		C.fill(0);
		D.fill(0);
	}
}

void ConvexHull::rotate(double yaw) {
	if (yaw == 0)
		return;

	double xOld, yOld;
	int size = x.rows();
	for(int i = 0; i < size; ++i){
		xOld = x(i);
		yOld = y(i);
		x(i) = (xOld*std::cos(yaw) - yOld*std::sin(yaw));
		y(i) = (xOld*std::sin(yaw) + yOld*std::cos(yaw));
	}
}

void ConvexHull::computeLinearSystem(const Foot & foot) {
	double dx,dy,dc,x1,y1,x2,y2;
	unsigned nbRows = x.rows();

	double sign;
	if(foot == LEFT){
		sign = 1.0;
	}else{
		sign = -1.0;
	}
	for( unsigned i=0; i<nbRows;++i ){
		y1 = y(i);
		y2 = y((i+1)%nbRows);
		x1 = x(i);
		x2 = x((i+1)%nbRows);

		dx = sign*(y1-y2);
		dy = sign*(x2-x1);
		dc = dx*x1+dy*y1;


		A(i) = dx;
		B(i) = dy;
		D(i) = dc;
	}
}
}


