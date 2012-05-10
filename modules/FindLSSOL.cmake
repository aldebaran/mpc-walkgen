set(prefix "LSSOL")
string(TOUPPER ${prefix} uprefix)

if(NOT ${uprefix}_FOUND)
  # check whether a foo-config.cmake or FooConfig.cmake file is available
  find_package(${prefix} ${${prefix}_FIND_VERSION} QUIET NO_MODULE)
endif()

if(NOT ${uprefix}_FOUND)
  # if not found yet, look at standard places
  find_path(${uprefix}_INCLUDE_DIRS "lssol/lssol.h")
  find_library(${uprefix}_LSSOL_LIBRARY "lssol")
  find_library(${uprefix}_LSSOL_C_LIBRARY "lssol_c")
  find_library(${uprefix}_F2C_LIBRARY "f2c")
  find_library(${uprefix}_BLAS_LIBRARY "blas")
  #find_library(${uprefix}_GFORTRAN_LIBRARY "gfortran" PATH_SUFFIXES
  #    "gcc/i686-linux-gnu/4.6" "gcc/x86_64-linux-gnu/4.6")
endif()

# Knowing the version of qpOASES is mandatory.
# If not found, the compilation flags and the #include<> may be wrong.
find_package_handle_standard_args(${prefix}
    REQUIRED_VARS ${uprefix}_INCLUDE_DIRS
    ${uprefix}_LSSOL_LIBRARY ${uprefix}_LSSOL_C_LIBRARY ${uprefix}_F2C_LIBRARY
    VERSION_VAR ${uprefix}_VERSION)

# publish
if(${uprefix}_FOUND)
  set("${uprefix}_FOUND" TRUE CACHE INTERNAL "" FORCE)
  set("${uprefix}_LIBRARIES"
      ${${uprefix}_LSSOL_LIBRARY} ${${uprefix}_LSSOL_C_LIBRARY}
      ${${uprefix}_F2C_LIBRARY} ${${uprefix}_BLAS_LIBRARY}
      gfortran
      CACHE INTERNAL "" FORCE)

  # Fill the information required by the macro PKG_CONFIG_USE_DEPENDENCY
  foreach(_dir ${${uprefix}_INCLUDE_DIRS})
    set("_incldir"  "${_incldir} -I${_dir}"
        CACHE INTERNAL "" FORCE)
  endforeach()
  set("${uprefix}_CFLAGS"  "${_incldir}"
      CACHE INTERNAL "" FORCE)
  set("${uprefix}_LDFLAGS" "${${uprefix}_LIBRARIES}"
      CACHE INTERNAL "" FORCE)
endif()
