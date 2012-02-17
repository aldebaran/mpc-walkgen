#include <mpc-walkgen/types.h>
#include <cmath>
using namespace MPCWalkgen;

int MPCData::iterationNumberFeedback(double firstIterationduration) const{
	return static_cast<int> (round(firstIterationduration / MPCSamplingPeriod)-1 );
}

int MPCData::nbIterationFeedback() const{
	return static_cast<int> (round(QPSamplingPeriod / MPCSamplingPeriod) );
}

int MPCData::nbIterationSimulation() const{
	return static_cast<int> (round(MPCSamplingPeriod / simSamplingPeriod) );
}

HipYawData::HipYawData(
	double lowerBound,
	double upperBound,
	double lowerVelocityBound,
	double upperVelocityBound,
	double lowerAccelerationBound,
	double upperAccelerationBound
): lowerBound_(lowerBound), upperBound_(upperBound)
, lowerVelocityBound_(lowerVelocityBound)
, upperVelocityBound_(upperVelocityBound)
, lowerAccelerationBound_(lowerAccelerationBound)
, upperAccelerationBound_(upperAccelerationBound)
{}


VelReference::Frame::Frame()
	:x(0),y(0),yaw(0)
	,xVec(1),yVec(1)
{}


VelReference::VelReference()
	:global()
	,local()
{}




SelectionMatrices::SelectionMatrices(const MPCData & generalData)
	:V(generalData.QPNbSamplings,generalData.QPNbSamplings)
	,VT(generalData.QPNbSamplings,generalData.QPNbSamplings)
	,VcX(generalData.QPNbSamplings)
	,VcY(generalData.QPNbSamplings)
	,Vf(generalData.QPNbSamplings,generalData.QPNbSamplings)
	,VcfX(generalData.QPNbSamplings)
	,VcfY(generalData.QPNbSamplings)
{}


QPPonderation::QPPonderation(int nb)
	:instantVelocity(nb)
	,CopCentering(nb)
	,JerkMin(nb)
{}



BodyState::BodyState(){
	reset();
}

void BodyState::reset(){
	x.fill(0);
	y.fill(0);
	z.fill(0);
	yaw.fill(0);
}





ConvexHull & ConvexHull::operator=(const ConvexHull & hull){
	x=hull.x;
	y=hull.y;
	A=hull.A;
	B=hull.B;
	C=hull.C;
	D=hull.D;

	return *this;
}

void ConvexHull::resize(int size){
	if (size!=x.rows()){
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
	for(int i=0;i<size;++i){
		xOld = x(i);
		yOld = y(i);
		x(i) = (xOld*std::cos(yaw) - yOld*std::sin(yaw));
		y(i) = (xOld*std::sin(yaw) + yOld*std::cos(yaw));
	}
}

void ConvexHull::computeLinearSystem(const Foot & foot){
	double dx,dy,dc,x1,y1,x2,y2;
	unsigned nbRows = x.rows();

	double sign;
	if(foot==LEFT){
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



void RelativeInequalities::resize(int rows, int cols){
	if (rows!=DX.rows() || cols!=DX.cols()){
		DX.setZero(rows, cols);
		DY.setZero(rows, cols);
		Dc.setZero(rows, cols);
	}else{
		DX.fill(0);
		DY.fill(0);
		Dc.fill(0);
	}
}
