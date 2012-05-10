set(prefix "QPOASES")
string(TOUPPER ${prefix} uprefix)

macro(find_qpoases_2)
  find_path(${uprefix}_INCLUDE_DIRS QProblem.hpp
      PATH_SUFFIXES qpoases)
  find_library(${uprefix}_LIBRARIES qpOASES)
endmacro()

macro(find_qpoases_3)
  find_path(${uprefix}_INCLUDE_DIRS qpOASES.hpp)
  find_library(${uprefix}_LIBRARIES qpOASES)
endmacro()

macro(find_version)
  # find QProblem.hpp in ${uprefix}_INCLUDE_DIRS and extract the qpOASES
  # from its doxygen version
  if(NOT "${uprefix}_VERSION")
    # Only extract the version if it is not already set
    set("${uprefix}_VERSION" "${uprefix}_VERSION-NOTFOUND")
    if(${uprefix}_INCLUDE_DIRS)
      find_file(_${uprefix}_QProblem_hpp
          "QProblem.hpp"
          PATHS  ${${uprefix}_INCLUDE_DIRS}
          PATH_SUFFIXES "qpOASES"
          NO_DEFAULT_PATH
          NO_CMAKE_ENVIRONMENT_PATH
          NO_CMAKE_PATH
          NO_SYSTEM_ENVIRONMENT_PATH
          NO_CMAKE_SYSTEM_PATH
          NO_CMAKE_FIND_ROOT_PATH)
      set_property(CACHE _${uprefix}_QProblem_hpp
          PROPERTY TYPE INTERNAL)
      file(STRINGS ${_${uprefix}_QProblem_hpp} _version_lines
          REGEX "[\\]version")
      if(NOT _${uprefix}_QProblem_hpp)
          message(FATAL_ERROR
              "FindQPOASES: could not find QProblem.hpp header")
      endif()
      list(GET _version_lines 0 _version_line)
      string(REGEX MATCH
          "[\\]version ([0123456789.]+)"
          _version_match "${_version_line}")
      if(CMAKE_MATCH_1)
        set("${uprefix}_VERSION" "${CMAKE_MATCH_1}")
      endif()
    endif()
  endif()
  set("${uprefix}_VERSION" "${${uprefix}_VERSION}" CACHE INTERNAL "" FORCE)
endmacro()

if(NOT ${uprefix}_FOUND)
  # check whether a foo-config.cmake or FooConfig.cmake file is available
  find_package(${prefix} ${${prefix}_FIND_VERSION} QUIET NO_MODULE)
endif()

if(NOT ${uprefix}_FOUND)
  # if not found yet, look at standard places
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
endif()

find_version()

# Knowing the version of qpOASES is mandatory.
# If not found, the compilation flags and the #include<> may be wrong.
find_package_handle_standard_args(${prefix}
    REQUIRED_VARS ${uprefix}_INCLUDE_DIRS ${uprefix}_LIBRARIES ${uprefix}_VERSION
    VERSION_VAR ${uprefix}_VERSION)

# publish
if(${uprefix}_FOUND)
  set("${uprefix}_FOUND" TRUE CACHE INTERNAL "" FORCE)

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
