#include "perturbation.h"


using namespace MPCWalkgen;

Perturbation::Perturbation(RigidBodySystem * robot)
        :robot_(robot)
{}

Perturbation::~Perturbation(){}

void Perturbation::applyForce(Axis axis, BodyType body, double f){
  applyAcc(axis, body, f/robot_->robotData().robotMass);
}

void Perturbation::applyAcc(Axis axis, BodyType body, double acc){
  BodyState newstate = robot_->body(body)->state();

  switch(axis){
    case X:
      newstate.x[2]+=acc;
    break;
    case Y:
      newstate.y[2]+=acc;
    break;
    case Z:
      newstate.z[2]+=acc;
    break;
    case Yaw:
      newstate.yaw[2]+=acc;
    break;
  }

}

void Perturbation::applyVel(Axis axis, BodyType body, double vel){
  BodyState newstate = robot_->body(body)->state();

  switch(axis){
    case X:
      newstate.x[1]+=vel;
    break;
    case Y:
      newstate.y[1]+=vel;
    break;
    case Z:
      newstate.z[1]+=vel;
    break;
    case Yaw:
      newstate.yaw[1]+=vel;
    break;
  }

}
