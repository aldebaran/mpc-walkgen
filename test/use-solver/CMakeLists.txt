string(TOLOWER ${CURRENT_QPSOLVER} lsolver)
add_definitions(" -DCURRENT_QPSOLVERTYPE=QPSOLVERTYPE_${CURRENT_QPSOLVER}")

############
#Benchs
############

############
#Units tests
############

############
#API tests
############
qi_create_test(test-humanoid-${lsolver}
  SRC api-test/test-humanoid.cpp ../../src/common/mpc-debug.cpp
  DEPENDS mpc-walkgen timer)

qi_create_test(test-zebulon-${lsolver}
  SRC api-test/test-zebulon.cpp ../../src/common/mpc-debug.cpp
  DEPENDS mpc-walkgen timer)
