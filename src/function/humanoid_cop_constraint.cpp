////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_cop_constraint.cpp
///\brief Implement the CoP constraint
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/function/humanoid_cop_constraint.h>
#include <mpc-walkgen/constant.h>
#include "../macro.h"

namespace MPCWalkgen
{
  template <typename Scalar>
  HumanoidCopConstraint<Scalar>::HumanoidCopConstraint(const LIPModel<Scalar>& lipModel,
                                               const HumanoidFeetSupervisor<Scalar>& feetSupervisor)
    :lipModel_(lipModel)
    ,feetSupervisor_(feetSupervisor)
    ,nbGeneralConstraints_(0)
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());

    function_.setZero(1);
    gradient_.setZero(1, 1);

    computeConstantPart();

    A_.setZero(1, 1);
    b_.setZero(1);
  }

  template <typename Scalar>
  HumanoidCopConstraint<Scalar>::~HumanoidCopConstraint()
  {}


  template <typename Scalar>
  int HumanoidCopConstraint<Scalar>::getNbConstraints()
  {
    computeNbGeneralConstraints();
    return nbGeneralConstraints_;
  }

  template <typename Scalar>
  const typename
  Type<Scalar>::VectorX& HumanoidCopConstraint<Scalar>::getFunction(const VectorX& x0)
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());
    assert(x0.rows() ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    computeGeneralConstraintsMatrices(x0.rows());

    function_.noalias() = A_*x0 + b_;
    return function_;
  }

  template <typename Scalar>
  const typename
  Type<Scalar>::MatrixX& HumanoidCopConstraint<Scalar>::getGradient(int sizeVec)
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());
    assert(sizeVec ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    computeGeneralConstraintsMatrices(sizeVec);

    gradient_ = A_;
    return gradient_;
  }

  template <typename Scalar>
  const typename
  Type<Scalar>::VectorX& HumanoidCopConstraint<Scalar>::getSupBounds(const VectorX& x0)
  {
    assert(x0.rows() ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    computeBoundsVectors(x0.segment(0, 2*lipModel_.getNbSamples()));

    return supBound_;
  }

  template <typename Scalar>
  const typename
  Type<Scalar>::VectorX& HumanoidCopConstraint<Scalar>::getInfBounds(const VectorX &x0)
  {
    assert(x0.rows() ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    computeBoundsVectors(x0.segment(0, 2*lipModel_.getNbSamples()));

    return infBound_;
  }

  template <typename Scalar>
  void HumanoidCopConstraint<Scalar>::computeNbGeneralConstraints()
  {
    nbGeneralConstraints_ = 0;

    for(int i = 0; i<lipModel_.getNbSamples(); ++i)
    {
      nbGeneralConstraints_+=
          feetSupervisor_.getCopConvexPolygon(i).getNbGeneralConstraints();
    }
  }


  template <typename Scalar>
  void HumanoidCopConstraint<Scalar>::computeGeneralConstraintsMatrices(int sizeVec)
  {
    computeNbGeneralConstraints();

    int N = lipModel_.getNbSamples();

    A_.setZero(nbGeneralConstraints_,
               sizeVec);
    b_.setConstant(nbGeneralConstraints_, Constant<Scalar>::MAXIMUM_BOUND_VALUE);

    // Number of general constraints at current time
    int nbGeneralConstraintsAtCurrentSample(0);
    // Sum of general constraints numbers since first sample
    int nbGeneralConstraintsSinceFirstSample(0);

    for(int i = 0; i<N; ++i)
    {
      const ConvexPolygon<Scalar>& cp = feetSupervisor_.getCopConvexPolygon(i);

      nbGeneralConstraintsAtCurrentSample = cp.getNbGeneralConstraints();

      for(int j = 0; j<nbGeneralConstraintsAtCurrentSample; ++j)
      {
        // Filling matrix A and vector b to create a general constraint of the form
        // AX + b <=0
        A_(nbGeneralConstraintsSinceFirstSample + j, i) =
            cp.getGeneralConstraintsMatrixCoefsForX()(j);

        A_(nbGeneralConstraintsSinceFirstSample + j, i + N) =
            cp.getGeneralConstraintsMatrixCoefsForY()(j);

        b_(nbGeneralConstraintsSinceFirstSample + j) =
            cp.getGeneralConstraintsConstantPart()(j);
      }

      nbGeneralConstraintsSinceFirstSample += nbGeneralConstraintsAtCurrentSample;
    }
  }

  template <typename Scalar>
  void HumanoidCopConstraint<Scalar>::computeBoundsVectors(const VectorX& x0)
  {
    int N = lipModel_.getNbSamples();

    assert(x0.rows()==2*N);

    for(int i = 0; i<N; ++i)
    {
      const ConvexPolygon<Scalar>& cp = feetSupervisor_.getCopConvexPolygon(i);

      supBound_(i) = cp.getXSupBound();
      supBound_(i + N) = cp.getYSupBound();
      infBound_(i) = cp.getXInfBound();
      infBound_(i + N) = cp.getYInfBound();
    }

    // As we optimize a variation dX_ of the QP variable X_ (such that X_ = x0 + dX_),
    // the bound constraints need to be translated of x0 too.
    supBound_ -= x0;
    infBound_ -= x0;
  }

  template <typename Scalar>
  void HumanoidCopConstraint<Scalar>::computeConstantPart()
  {
    int N = lipModel_.getNbSamples();

    supBound_.setConstant(2*N, Constant<Scalar>::MAXIMUM_BOUND_VALUE);
    infBound_.setConstant(2*N, -Constant<Scalar>::MAXIMUM_BOUND_VALUE);
  }

  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(HumanoidCopConstraint);
}
