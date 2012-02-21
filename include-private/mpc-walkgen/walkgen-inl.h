#ifndef WALKGEN_INL_H_
#define WALKGEN_INL_H_

namespace MPCWalkgen {

inline const SupportState & Walkgen::currentSupportState() const {
	return robot_->currentSupport(); }
inline double Walkgen::comHeight() const
{ return robot_->robotData().CoMHeight; }
inline void Walkgen::comHeight(double d)
{ robot_->robotData().CoMHeight=d; }
inline double Walkgen::freeFlyingFootMaxHeight() const
{ return robot_->robotData().freeFlyingFootMaxHeight; }
inline void Walkgen::freeFlyingFootMaxHeight(double d)
{ robot_->robotData().freeFlyingFootMaxHeight = d; }

const BodyState & Walkgen::bodyState(BodyType body)const{
	return robot_->body(body)->state();
}
void Walkgen::bodyState(BodyType body, const BodyState & state){
	robot_->body(body)->state(state);
}

}

#endif /* WALKGEN_INL_H_ */
