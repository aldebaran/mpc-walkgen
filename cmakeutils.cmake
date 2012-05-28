# create_test: create an executable and declare it as a test.
#
# create_test(name
#   SRC src0.cpp src1.cpp
#   INTDEPS intdep0 intdep1
#   EXTDEPS extdep0 extdep1
#   ARGUMENTS arg0 arg1)
#
# - name the name of the test and the target
# - SRC: sources of the test
# - INTDEPS: internal dependencies of the test (cmake targets)
# - EXTDEPS: external dependencies of the test (libraries)
# - ARGUMENTS: arguments to be passed to the executable
#
function(create_test name)
  cmake_parse_arguments(ARG "" "" "SRC;INTDEPS;EXTDEPS;ARGUMENTS" ${ARGN})

  # create the test target
  add_executable(${name} ${ARG_SRC} ${ARG_UNPARSED_ARGUMENTS})

  #handle dependencies
  foreach(dep ${ARG_INTDEPS})
    add_dependencies(${name} ${ARG_INTDEPS})
    target_link_libraries(${name} ${ARG_INTDEPS})
  endforeach()
  foreach(dep ${ARG_EXTDEPS})
    PKG_CONFIG_USE_DEPENDENCY(${name} ${dep})
  endforeach(dep)

  # avoid the Debug/Release subdirs in VS, so that we can find the executable
  SET(_output_directory "${CMAKE_CURRENT_BINARY_DIR}")
  SET_TARGET_PROPERTIES("${name}" PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${_output_directory}"
      RUNTIME_OUTPUT_DIRECTORY_RELEASE "${_output_directory}"
      RUNTIME_OUTPUT_DIRECTORY         "${_output_directory}"
  )
  # Now that all configs end up in the same directory, we need to avoid name
  # clashes
  if(MSVC)
    # always postfix debug lib/bin with _d ...
    SET_TARGET_PROPERTIES("${name}" PROPERTIES DEBUG_POSTFIX "_d")
  endif()

  set(_exec_path ${_output_directory}/${name})
  if(MSVC AND "${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    set(_exec_path ${_exec_path}_d)
  endif()

  # add the test
  add_test(${name} "${_exec_path}" ${ARG_ARGUMENTS})
endfunction()
