#pragma once
#ifndef _MPC_WALKGEN_API_HPP_
#define _MPC_WALKGEN_API_HPP_

// code copied from libqi
#if defined _WIN32 || defined __CYGWIN__
#  define QI_EXPORT_API __declspec(dllexport)
#  if defined _WINDLL
#    define QI_IMPORT_API __declspec(dllimport)
#  else
#    define QI_IMPORT_API
#  endif
#elif __GNUC__ >= 4
#  define QI_EXPORT_API __attribute__ ((visibility("default")))
#  define QI_IMPORT_API __attribute__ ((visibility("default")))
#else
#  define QI_EXPORT_API
#  define QI_IMPORT_API
#endif

// mpc_walkgen_EXPORTS is defined by the build system, only when building the
// library
#ifdef mpc_walkgen_EXPORTS
# define MPC_WALKGEN_API QI_EXPORT_API
#else
# define MPC_WALKGEN_API QI_IMPORT_API
#endif

#endif  // _MPC_WALKGEN_API_HPP_
