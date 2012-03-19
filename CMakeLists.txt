cmake_minimum_required(VERSION 2.8)

option(use_qibuild OFF "Do you want to use qibuild to build your package?")

if(use_qibuild)
  include(CMakeLists-qibuild.txt)
else(use_qibuild)
  include(CMakeLists-jrlcmake.txt)
endif(use_qibuild)

