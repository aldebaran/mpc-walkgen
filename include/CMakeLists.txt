
# Define the set of headers to be installed
# Do *not* name it ${PROJECT_NAME}_HEADERS to prevent jrlcmake to try
# installing them automatically (it would mix up the subdirectories)
set(_HEADERS
  mpc-walkgen/common/api.h
  mpc-walkgen/common/sharedpgtypes.h
  mpc-walkgen/humanoid/walkgen-abstract.h
  mpc-walkgen/humanoid/sharedpgtypes.h
  mpc-walkgen/zebulon/walkgen-abstract.h
  mpc-walkgen/zebulon/sharedpgtypes.h
)

qi_install_header("${_HEADERS}" KEEP_RELATIVE_PATHS)
