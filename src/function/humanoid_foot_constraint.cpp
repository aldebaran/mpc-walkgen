////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_foot_constraint.cpp
///\brief Implement the foot constraints
///\author de Gourcuff Martin
///\date 12/07/13
///
////////////////////////////////////////////////////////////////////////////////

#include "humanoid_foot_constraint.h"

namespace MPCWalkgen
{
  HumanoidFootConstraint::HumanoidFootConstraint(const LIPModel& lipModel,
                                                 const HumanoidFeetSupervisor& feetSupervisor)
    :lipModel_(lipModel)
    ,feetSupervisor_(feetSupervisor)
    ,nbGeneralConstraints_(0)
  {
    assert(std::abs(feetSupervisor_.getSamplingPeriod()
                    - lipModel_.getSamplingPeriod())<EPSILON);
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());

    function_.setZero(1);
    gradient_.setZero(1, 1);

    supBound_.setZero(1);
    infBound_.setZero(1);

    A_.setZero(1, 1);
    b_.setZero(1);
  }

  HumanoidFootConstraint::~HumanoidFootConstraint()
  {}


  unsigned int HumanoidFootConstraint::getNbConstraints()
  {
    xComputeNbGeneralConstraints();
    return nbGeneralConstraints_;
  }

  const VectorX& HumanoidFootConstraint::getFunction(const VectorX& x0)
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());
    assert(x0.rows() ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

      xComputeGeneralConstraintsMatrices(x0.rows());

    function_.noalias() = A_*x0 + b_;
    return function_;
  }

  const MatrixX& HumanoidFootConstraint::getGradient(unsigned int sizeVec)
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());
    assert(sizeVec ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

      xComputeGeneralConstraintsMatrices(sizeVec);

    gradient_ = A_;
    return gradient_;
  }

  const VectorX& HumanoidFootConstraint::getSupBounds()
  {
      xComputeBoundsVectors();

    return supBound_;
  }

  const VectorX& HumanoidFootConstraint::getInfBounds()
  {
      xComputeBoundsVectors();

    return infBound_;
  }

  void HumanoidFootConstraint::xComputeNbGeneralConstraints()
  {
    nbGeneralConstraints_ = 0;
    for(unsigned int i = 0; i<lipModel_.getNbSamples(); ++i)
    {
      nbGeneralConstraints_+=
          feetSupervisor_.getCopConvexPolygonVec()[i].getNbGeneralConstraints();
    }
  }


  void HumanoidFootConstraint::xComputeGeneralConstraintsMatrices(unsigned int sizeVec)
  {
    xComputeNbGeneralConstraints();

    unsigned int M = feetSupervisor_.getNbPreviewedSteps();

    A_.setZero(nbGeneralConstraints_,
               sizeVec);
    b_.setConstant(nbGeneralConstraints_, std::numeric_limits<Scalar>::max());


    //Number of general constraints at current step
    unsigned int nbGeneralConstraintsAtCurrentStep(0);
    //Sum of general constraints numbers since first step
    unsigned int nbGeneralConstraintsSinceFirstStep(0);

    for(unsigned int i = 0; i<M; ++i)
    {
      nbGeneralConstraintsAtCurrentStep =
          feetSupervisor_.getCopConvexPolygonVec()[i].getNbGeneralConstraints();

      for(unsigned int j = 0; j<nbGeneralConstraintsAtCurrentStep; ++j)
      {
        // Filling matrix A and vector b to create a general constraint of the form
        // AX + b <=0
        A_(nbGeneralConstraintsSinceFirstStep + j, i) =
            feetSupervisor_.getCopConvexPolygonVec()[i]
            .getGeneralConstraintsMatrixCoefsForX()(j);

        A_(nbGeneralConstraintsSinceFirstStep + j, i + M) =
            feetSupervisor_.getCopConvexPolygonVec()[i]
            .getGeneralConstraintsMatrixCoefsForY()(j);

        b_(nbGeneralConstraintsSinceFirstStep + j) =
            feetSupervisor_.getCopConvexPolygonVec()[i]
            .getGeneralConstraintsConstantPart()(j);
      }

      nbGeneralConstraintsSinceFirstStep += nbGeneralConstraintsAtCurrentStep;
    }
  }

  void HumanoidFootConstraint::xComputeBoundsVectors()
  {
    unsigned int M = feetSupervisor_.getNbPreviewedSteps();

    supBound_.setConstant(2*M, std::numeric_limits<Scalar>::max());
    infBound_.setConstant(2*M, -std::numeric_limits<Scalar>::max());

    for(unsigned int i = 0; i<M; ++i)
    {
      supBound_(i) = feetSupervisor_.getCopConvexPolygonVec()[i].getXSupBound();
      supBound_(i + M) = feetSupervisor_.getCopConvexPolygonVec()[i].getYSupBound();
      infBound_(i) = feetSupervisor_.getCopConvexPolygonVec()[i].getXInfBound();
      infBound_(i + M) = feetSupervisor_.getCopConvexPolygonVec()[i].getYInfBound();
    }
  }

}
