#include "types.h"


using namespace MPCWalkgen;


Reference::Frame::Frame()
        :x(1),y(1),yaw(1)
{
  x.fill(0);
  y.fill(0);
  yaw.fill(0);
}

void Reference::Frame::resize(int size){
  x.setZero(size);
  y.setZero(size);
  yaw.setZero(size);
}


Reference::Reference()
	:global()
	,local()
{}

void Reference::resize(int size){
  global.resize(size);
  local.resize(size);
}
