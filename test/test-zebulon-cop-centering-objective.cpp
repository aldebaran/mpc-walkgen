////////////////////////////////////////////////////////////////////////////////
///
///\file test-zebulon-cop-centering-objective.cpp
///\brief Test the Zebulon CoP centering objective function
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include "mpc_walkgen_gtest.h"
#include <mpc-walkgen/model/zebulon_base_model.h>
#include <mpc-walkgen/model/lip_model.h>
#include <mpc-walkgen/function/zebulon_cop_centering_objective.h>

TYPED_TEST(MpcWalkgenTest, functionValue)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);

  LIPModel<TypeParam> m1;
  BaseModel<TypeParam> m2;
  CopCenteringObjective<TypeParam> obj(m1, m2);

  VectorX jerkInit(4);
  jerkInit(0) = 1.0;
  jerkInit(1) = 1.0;
  jerkInit(2) = 1.0;
  jerkInit(3) = 1.0;

  VectorX copRef(2);
  copRef(0) = 2.0;
  copRef(1) = 2.0;
  obj.setCopRefInLocalFrame(copRef);


  VectorX baseState(3);
  baseState.fill(0.0);
  baseState(0) = -1.5;
  m2.setStateX(baseState);
  baseState(0) = 0.25;
  m2.setStateY(baseState);

  VectorX comState(3);
  comState.fill(0.0);
  comState(0) = -1.0;
  m1.setStateX(comState);
  comState(0) = 0.5;
  m1.setStateY(comState);

  ASSERT_NEAR(obj.getGradient(jerkInit)(0,0), -0.1036931, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getGradient(jerkInit)(1,0), -0.1198756, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getGradient(jerkInit)(2,0), 0.2669894, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getGradient(jerkInit)(3,0), 0.3086561, Constant<TypeParam>::EPSILON);


  ASSERT_NEAR(obj.getHessian()(0,0), 0.00418996, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(0,1), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(0,2), -0.01078831, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(0,3), 0.0, Constant<TypeParam>::EPSILON);

  ASSERT_NEAR(obj.getHessian()(1,0), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(1,1), 0.00418996, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(1,2), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(1,3), -0.01078831, Constant<TypeParam>::EPSILON);

  ASSERT_NEAR(obj.getHessian()(2,0), -0.01078831, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(2,1), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(2,2), 0.02777778, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(2,3), 0.0, Constant<TypeParam>::EPSILON);

  ASSERT_NEAR(obj.getHessian()(3,0), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(3,1), -0.01078831, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(3,2), 0.0, Constant<TypeParam>::EPSILON);
  ASSERT_NEAR(obj.getHessian()(3,3), 0.02777778, Constant<TypeParam>::EPSILON);

}

TYPED_TEST(MpcWalkgenTest, sizeOfvalues)
{
  using namespace MPCWalkgen;
  TEMPLATE_TYPEDEF(TypeParam);

  int nbSamples = 3;
  TypeParam samplingPeriod = 1.0;
  bool autoCompute = true;
  LIPModel<TypeParam> m1(nbSamples, samplingPeriod, autoCompute);
  BaseModel<TypeParam> m2(nbSamples, samplingPeriod, autoCompute);
  CopCenteringObjective<TypeParam> obj(m1, m2);

  VectorX jerkInit(4*nbSamples);
  jerkInit.fill(0.0);

  ASSERT_EQ(obj.getHessian().rows(), 4*nbSamples);
  ASSERT_EQ(obj.getHessian().cols(), 4*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).rows(), 4*nbSamples);
  ASSERT_EQ(obj.getGradient(jerkInit).cols(), 1);
}


TYPED_TEST(MpcWalkgenTest, coprefInWorldFrame)
{
    using namespace MPCWalkgen;
    TEMPLATE_TYPEDEF(TypeParam);

    int nbSamples = 5;
    TypeParam samplingPeriod = 1.0;

    bool autoCompute = true;
    LIPModel<TypeParam> m1(nbSamples, samplingPeriod, autoCompute);
    BaseModel<TypeParam> m2(nbSamples, samplingPeriod, autoCompute);
    CopCenteringObjective<TypeParam> obj(m1, m2);

    VectorX ze(4*nbSamples);
    ze.fill(0);

    VectorX jerk(4*nbSamples);
    jerk.segment(0, 2*nbSamples).fill(1.0);
    jerk.segment(2*nbSamples, 2*nbSamples).fill(2.0);

    const LinearDynamic<TypeParam>& comPosDynamic = m1.getComPosLinearDynamic();
    const LinearDynamic<TypeParam>& comAccDynamic = m1.getComAccLinearDynamic();
    const LinearDynamic<TypeParam>& basePosDynamic = m2.getBasePosLinearDynamic();
    const LinearDynamic<TypeParam>& baseAccDynamic = m2.getBaseAccLinearDynamic();

    VectorX comPosX = comPosDynamic.U * jerk.segment(0, nbSamples) +
                      comPosDynamic.S * m1.getStateX();
    VectorX comPosY = comPosDynamic.U * jerk.segment(nbSamples, nbSamples) +
                      comPosDynamic.S * m1.getStateY();
    VectorX basePosX = basePosDynamic.U * jerk.segment(2*nbSamples, nbSamples) +
                       basePosDynamic.S * m2.getStateX();
    VectorX basePosY = basePosDynamic.U * jerk.segment(3*nbSamples, nbSamples) +
                       basePosDynamic.S * m2.getStateY();

    VectorX comAccX = comAccDynamic.U * jerk.segment(0, nbSamples) +
                      comAccDynamic.S * m1.getStateX();
    VectorX comAccY = comAccDynamic.U * jerk.segment(nbSamples, nbSamples) +
                      comAccDynamic.S * m1.getStateY();
    VectorX baseAccX = baseAccDynamic.U * jerk.segment(2*nbSamples, nbSamples) +
                       baseAccDynamic.S * m2.getStateX();
    VectorX baseAccY = baseAccDynamic.U * jerk.segment(3*nbSamples, nbSamples) +
                       baseAccDynamic.S * m2.getStateY();

    VectorX comPos(2*nbSamples);
    comPos << comPosX, comPosY;

    VectorX basePos(2*nbSamples);
    basePos << basePosX, basePosY;

    VectorX comAcc(2*nbSamples);
    comAcc << comAccX, comAccY;

    VectorX baseAcc(2*nbSamples);
    baseAcc << baseAccX, baseAccY;

    obj.setCopRefInWorldFrame(comPos, comAcc, basePos, baseAcc);
    MatrixX vecObj1 = obj.getGradient(ze);


    const LinearDynamic<TypeParam>& comCopXDynamic = m1.getCopXLinearDynamic();
    const LinearDynamic<TypeParam>& comCopYDynamic = m1.getCopYLinearDynamic();
    const LinearDynamic<TypeParam>& baseCopXDynamic = m2.getCopXLinearDynamic();
    const LinearDynamic<TypeParam>& baseCopYDynamic = m2.getCopYLinearDynamic();

    VectorX copX = comCopXDynamic.U * jerk.segment(0, nbSamples) +
                   comCopXDynamic.S * m1.getStateX() +
                   baseCopXDynamic.U * jerk.segment(2*nbSamples, nbSamples) +
                   baseCopXDynamic.S * m2.getStateX();
    VectorX copY = comCopYDynamic.U * jerk.segment(nbSamples, nbSamples) +
                   comCopYDynamic.S * m1.getStateY() +
                   baseCopYDynamic.U * jerk.segment(3*nbSamples, nbSamples) +
                   baseCopYDynamic.S * m2.getStateY();

    VectorX cop(2*nbSamples);
    cop << copX-basePosX, copY-basePosY;

    obj.setCopRefInLocalFrame(cop);

    MatrixX vecObj2 = obj.getGradient(ze);

    for(int i=0; i<vecObj1.rows(); i++){
        for(int j=0; j<vecObj1.cols(); j++){
            ASSERT_NEAR(vecObj1(i, j), vecObj2(i, j), 1e-4f);
        }
    }
}
