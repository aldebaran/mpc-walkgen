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


  unsigned int HumanoidCopConstraint::getNbConstraints()
  {
    xComputeNbGeneralConstraints();
    return nbGeneralConstraints_;
  }

  const VectorX& HumanoidCopConstraint::getFunction(const VectorX& x0)
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());
    assert(x0.rows() ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    xComputeGeneralConstraintsMatrices(x0.rows());

    function_.noalias() = A_*x0 + b_;
    return function_;
  }

  const MatrixX& HumanoidCopConstraint::getGradient(unsigned int sizeVec)
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());
    assert(sizeVec ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    xComputeGeneralConstraintsMatrices(sizeVec);

    gradient_ = A_;
    return gradient_;
  }

  const VectorX& HumanoidCopConstraint::getSupBounds()
  {
    xComputeBoundsVectors();

    return supBound_;
  }

  const VectorX& HumanoidCopConstraint::getInfBounds()
  {
    xComputeBoundsVectors();

    return infBound_;
  }

  void HumanoidCopConstraint::xComputeNbGeneralConstraints()
  {
    nbGeneralConstraints_ = 0;
    for(unsigned int i = 0; i<lipModel_.getNbSamples(); ++i)
    {
      nbGeneralConstraints_+=
          feetSupervisor_.getCopConvexPolygonVec()[i].getNbGeneralConstraints();
    }
  }


  void HumanoidCopConstraint::xComputeGeneralConstraintsMatrices(unsigned int sizeVec)
  {
    xComputeNbGeneralConstraints();

    unsigned int N = lipModel_.getNbSamples();

    A_.setZero(nbGeneralConstraints_,
               sizeVec);
    b_.setConstant(nbGeneralConstraints_, std::numeric_limits<Scalar>::max());

    // Number of general constraints at current time
    unsigned int nbGeneralConstraintsAtCurrentSample(0);
    // Sum of general constraints numbers since first sample
    unsigned int nbGeneralConstraintsSinceFirstSample(0);
    // Step number at ith sample (i being the iteration variable in next for loop)
    unsigned int numStepAtSample(0);

    for(unsigned int i = 0; i<N; ++i)
    {
      numStepAtSample = feetSupervisor_.sampleToStep(i);

      nbGeneralConstraintsAtCurrentSample =
          feetSupervisor_.getCopConvexPolygonVec()[numStepAtSample]
          .getNbGeneralConstraints();

      for(unsigned int j = 0; j<nbGeneralConstraintsAtCurrentSample; ++j)
      {
       // Filling matrix A and vector b to create a general constraint of the form
       // AX + b <=0
        A_(nbGeneralConstraintsSinceFirstSample + j, i) =
            feetSupervisor_.getCopConvexPolygonVec()[numStepAtSample]
            .getGeneralConstraintsMatrixCoefsForX()(j);

        A_(nbGeneralConstraintsSinceFirstSample + j, i + N) =
            feetSupervisor_.getCopConvexPolygonVec()[numStepAtSample]
            .getGeneralConstraintsMatrixCoefsForY()(j);

        b_(nbGeneralConstraintsSinceFirstSample + j) =
            feetSupervisor_.getCopConvexPolygonVec()[numStepAtSample]
            .getGeneralConstraintsConstantPart()(j);
      }

      nbGeneralConstraintsSinceFirstSample += nbGeneralConstraintsAtCurrentSample;
    }
  }

  void HumanoidCopConstraint::xComputeBoundsVectors()
  {
    unsigned int N = lipModel_.getNbSamples();
    unsigned int numStepAtSample(0);

    for(unsigned int i = 0; i<N; ++i)
    {

      numStepAtSample = feetSupervisor_.sampleToStep(i);

      supBound_(i) =
          feetSupervisor_.getCopConvexPolygonVec()[numStepAtSample]
          .getXSupBound();
      supBound_(i + N) =
          feetSupervisor_.getCopConvexPolygonVec()[numStepAtSample]
          .getYSupBound();
      infBound_(i) =
          feetSupervisor_.getCopConvexPolygonVec()[numStepAtSample]
          .getXInfBound();
      infBound_(i + N) =
          feetSupervisor_.getCopConvexPolygonVec()[numStepAtSample]
          .getYInfBound();
    }
  }

  void HumanoidCopConstraint::computeConstantPart()
  {
    unsigned int N = lipModel_.getNbSamples();

    supBound_.setConstant(2*N, std::numeric_limits<Scalar>::max());
    infBound_.setConstant(2*N, -std::numeric_limits<Scalar>::max());
  }

}
