string(TOLOWER ${CURRENT_QPSOLVER} lsolver)
add_definitions(" -DCURRENT_QPSOLVERTYPE=QPSOLVERTYPE_${CURRENT_QPSOLVER}")

create_test(test-humanoid-${lsolver}
  SRC test-humanoid.cpp ../../src/common/mpc-debug.cpp
  INTDEPS mpc-walkgen timer)

create_test(test-zebulon-${lsolver}
  SRC test-zebulon.cpp ../../src/common/mpc-debug.cpp
  INTDEPS mpc-walkgen timer)
