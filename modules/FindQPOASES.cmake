set(prefix "QPOASES")
string(TOUPPER ${prefix} uprefix)

macro(find_qpoases_2)
  find_path(${uprefix}_INCLUDE_DIRS QProblem.hpp
      PATH_SUFFIXES qpoases)
  find_library(${uprefix}_LIBRARIES qpoases)
  set(${uprefix}_VERSION 2.0) # todo: find the version in the headers
endmacro()

macro(find_qpoases_3)
  find_path(${uprefix}_INCLUDE_DIRS qpOASES.hpp)
  find_library(${uprefix}_LIBRARIES qpOASES)
  set(${uprefix}_VERSION 3.0) # todo: find the version  in the headers
endmacro()

if(DEFINED ${prefix}_FIND_VERSION)
  if(${prefix}_FIND_VERSION VERSION_LESS 3.0)
    find_qpoases_2()
  else()
    find_qpoases_3()
  endif()
else()
  find_qpoases_2()
  if(NOT (${uprefix}_LIBRARIES AND ${uprefix}_INCLUDE_DIRS))
    find_qpoases_3()
  endif()
endif()

find_package_handle_standard_args(${prefix}
    REQUIRED_VARS ${uprefix}_INCLUDE_DIRS ${uprefix}_LIBRARIES
    VERSION_VAR ${uprefix}_VERSION)

# publish
if(${uprefix}_FOUND)
  set("${uprefix}_FOUND" TRUE CACHE INTERNAL "" FORCE)
  set("${uprefix}_INCLUDE_DIRS" "${${uprefix}_INCLUDE_DIRS}" CACHE PATH "")
  set("${uprefix}_LIBRARIES" "${${uprefix}_LIBRARIES}" CACHE PATH "")
  set("${uprefix}_VERSION" "${${uprefix}_VERSION}" CACHE STRING "")

  # Fill the information required by the macro PKG_CONFIG_USE_DEPENDENCY
  set("${uprefix}_CFLAGS"  "-I${${uprefix}_INCLUDE_DIRS}" CACHE INTERNAL "" FORCE)
  set("${uprefix}_LDFLAGS" "${${uprefix}_LIBRARIES}" CACHE INTERNAL "" FORCE)
endif()

