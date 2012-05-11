set(prefix "LSSOL")
string(TOUPPER ${prefix} uprefix)

if(NOT ${uprefix}_FOUND)
  # check whether a foo-config.cmake or FooConfig.cmake file is available
  find_package(${prefix} ${${prefix}_FIND_VERSION} QUIET NO_MODULE)
endif()

if(NOT ${uprefix}_FOUND)
  # if not found yet, look at standard places
  find_path(${uprefix}_INCLUDE_DIRS "lssol/lssol.h")
  find_library(lssol_c_LIB "lssol_c")
  find_library(lssol_LIB "lssol")
  find_library(blas_LIB "blas")
  find_library(f2c_LIB "f2c")
  #find_library(${uprefix}_GFORTRAN_LIBRARY "gfortran" PATH_SUFFIXES
  #    "gcc/i686-linux-gnu/4.6" "gcc/x86_64-linux-gnu/4.6")
endif()

# Knowing the version of qpOASES is mandatory.
# If not found, the compilation flags and the #include<> may be wrong.
find_package_handle_standard_args(${prefix}
    REQUIRED_VARS ${uprefix}_INCLUDE_DIRS
    lssol_c_LIB lssol_LIB blas_LIB f2c_LIB
    VERSION_VAR ${uprefix}_VERSION)

# publish
if(${uprefix}_FOUND)
  set("${uprefix}_FOUND" TRUE CACHE INTERNAL "" FORCE)
  # gfortran cannot be found by find_library as is lies in
  # /usr/lib/gcc/<arch>/<gcc_version>/
  # So we let the linker find it.
  # Also, beware the *link order matters*
  set("${uprefix}_LIBRARIES"
      ${lssol_c_LIB} ${lssol_LIB} ${blas_LIB} ${f2c_LIB}
      gfortran
      CACHE INTERNAL "" FORCE)

  # Fill the information required by the macro PKG_CONFIG_USE_DEPENDENCY
  foreach(_dir ${${uprefix}_INCLUDE_DIRS})
    set("_incldir"  "${_incldir} -I${_dir}"
        CACHE INTERNAL "" FORCE)
  endforeach()
  set("${uprefix}_CFLAGS"  "${_incldir}"
      CACHE INTERNAL "" FORCE)
  #set("${uprefix}_LDFLAGS" "${${uprefix}_LIBRARIES}"
  #    CACHE INTERNAL "" FORCE)
endif()
