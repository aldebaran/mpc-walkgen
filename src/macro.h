////////////////////////////////////////////////////////////////////////////////
///
///\file macro.h
///\brief macro to explicitly instantiate template
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#ifndef MPC_WALKGEN_MACRO_H
#define MPC_WALKGEN_MACRO_H

#define MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(className) \
template class MPC_WALKGEN_API className<double>;\
template class MPC_WALKGEN_API className<float>

#endif
