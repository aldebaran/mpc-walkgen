#include <mpc-walkgen/types.h>
#include <cmath>
using namespace MPCWalkgen;

HipYawData::HipYawData(
	double lowerBound,
	double upperBound,
	double lowerVelocityBound,
	double upperVelocityBound,
	double lowerAccelerationBound,
	double upperAccelerationBound
): lowerBound(lowerBound), upperBound(upperBound)
, lowerVelocityBound(lowerVelocityBound)
, upperVelocityBound(upperVelocityBound)
, lowerAccelerationBound(lowerAccelerationBound)
, upperAccelerationBound(upperAccelerationBound)
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
