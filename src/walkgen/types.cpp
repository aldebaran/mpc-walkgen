#include <mpc-walkgen/types.h>
#include <cmath>
using namespace MPCWalkgen;


VelReference::Frame::Frame()
        :x(1),y(1),yaw(1)
{
  x.fill(0);
  y.fill(0);
  yaw.fill(0);
}

void VelReference::Frame::resize(int size){
  x.setZero(size);
  y.setZero(size);
  yaw.setZero(size);
}


VelReference::VelReference()
	:global()
	,local()
{}

void VelReference::resize(int size){
  global.resize(size);
  local.resize(size);
}


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
