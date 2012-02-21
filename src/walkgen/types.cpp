#include <mpc-walkgen/types.h>
#include <cmath>
using namespace MPCWalkgen;


VelReference::Frame::Frame()
	:x(0),y(0),yaw(0)
	,xVec(1),yVec(1)
{}

VelReference::VelReference()
	:global()
	,local()
{}

SelectionMatrices::SelectionMatrices(const MPCData & generalData)
	:V(generalData.nbSamplesQP,generalData.nbSamplesQP)
	,VT(generalData.nbSamplesQP,generalData.nbSamplesQP)
	,VcX(generalData.nbSamplesQP)
	,VcY(generalData.nbSamplesQP)
	,Vf(generalData.nbSamplesQP,generalData.nbSamplesQP)
	,VcfX(generalData.nbSamplesQP)
	,VcfY(generalData.nbSamplesQP)
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
