#include "common-types.h"
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
