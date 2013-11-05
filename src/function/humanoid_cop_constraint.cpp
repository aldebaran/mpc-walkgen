////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_cop_constraint.cpp
///\brief Implement the CoP constraint
///\author de Gourcuff Martin
///\date 12/07/13
///
////////////////////////////////////////////////////////////////////////////////

#include "humanoid_cop_constraint.h"

namespace MPCWalkgen
{
  HumanoidCopConstraint::HumanoidCopConstraint(const LIPModel& lipModel,
                                               const HumanoidFeetSupervisor& feetSupervisor)
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

  HumanoidCopConstraint::~HumanoidCopConstraint()
  {}


  int HumanoidCopConstraint::getNbConstraints()
  {
    computeNbGeneralConstraints();
    return nbGeneralConstraints_;
  }

  const VectorX& HumanoidCopConstraint::getFunction(const VectorX& x0)
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());
    assert(x0.rows() ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    computeGeneralConstraintsMatrices(x0.rows());

    function_.noalias() = A_*x0 + b_;
    return function_;
  }

  const MatrixX& HumanoidCopConstraint::getGradient(int sizeVec)
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());
    assert(sizeVec ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    computeGeneralConstraintsMatrices(sizeVec);

    gradient_ = A_;
    return gradient_;
  }

  const VectorX& HumanoidCopConstraint::getSupBounds(const VectorX& x0)
  {
    assert(x0.rows() ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    computeBoundsVectors(x0.segment(0, 2*lipModel_.getNbSamples()));

    return supBound_;
  }

  const VectorX& HumanoidCopConstraint::getInfBounds(const VectorX &x0)
  {
    assert(x0.rows() ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    computeBoundsVectors(x0.segment(0, 2*lipModel_.getNbSamples()));

    return infBound_;
  }

  void HumanoidCopConstraint::computeNbGeneralConstraints()
  {
    nbGeneralConstraints_ = 0;

    for(int i = 0; i<lipModel_.getNbSamples(); ++i)
    {
      nbGeneralConstraints_+=
          feetSupervisor_.getCopConvexPolygon(i).getNbGeneralConstraints();
    }
  }


  void HumanoidCopConstraint::computeGeneralConstraintsMatrices(int sizeVec)
  {
    computeNbGeneralConstraints();

    int N = lipModel_.getNbSamples();

    A_.setZero(nbGeneralConstraints_,
               sizeVec);
    b_.setConstant(nbGeneralConstraints_, MAXIMUM_BOUND_VALUE);

    // Number of general constraints at current time
    int nbGeneralConstraintsAtCurrentSample(0);
    // Sum of general constraints numbers since first sample
    int nbGeneralConstraintsSinceFirstSample(0);

    for(int i = 0; i<N; ++i)
    {
      const ConvexPolygon& cp = feetSupervisor_.getCopConvexPolygon(i);

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

  void HumanoidCopConstraint::computeBoundsVectors(const VectorX& x0)
  {
    int N = lipModel_.getNbSamples();

    assert(x0.rows()==2*N);

    for(int i = 0; i<N; ++i)
    {
      const ConvexPolygon& cp = feetSupervisor_.getCopConvexPolygon(i);

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

  void HumanoidCopConstraint::computeConstantPart()
  {
    int N = lipModel_.getNbSamples();

    supBound_.setConstant(2*N, MAXIMUM_BOUND_VALUE);
    infBound_.setConstant(2*N, -MAXIMUM_BOUND_VALUE);
  }

}
