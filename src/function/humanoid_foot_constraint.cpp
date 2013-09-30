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


  int HumanoidFootConstraint::getNbConstraints()
  {
    computeNbGeneralConstraints();
    return nbGeneralConstraints_;
  }

  const VectorX& HumanoidFootConstraint::getFunction(const VectorX& x0)
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());
    assert(x0.rows() ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

      computeGeneralConstraintsMatrices(x0.rows());

    function_.noalias() = A_*x0 + b_;
    return function_;
  }

  const MatrixX& HumanoidFootConstraint::getGradient(int sizeVec)
  {
    assert(feetSupervisor_.getNbSamples() == lipModel_.getNbSamples());
    assert(sizeVec ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

      computeGeneralConstraintsMatrices(sizeVec);

    gradient_ = A_;
    return gradient_;
  }

  const VectorX& HumanoidFootConstraint::getSupBounds(const VectorX& x0)
  {
    assert(x0.rows() ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    computeBoundsVectors(x0.segment(2*lipModel_.getNbSamples(),
                                     2*feetSupervisor_.getNbPreviewedSteps()));

    return supBound_;
  }

  const VectorX& HumanoidFootConstraint::getInfBounds(const VectorX& x0)
  {
    assert(x0.rows() ==
           2*lipModel_.getNbSamples() + 2*feetSupervisor_.getNbPreviewedSteps());

    computeBoundsVectors(x0.segment(2*lipModel_.getNbSamples(),
                                     2*feetSupervisor_.getNbPreviewedSteps()));

    return infBound_;
  }

  void HumanoidFootConstraint::computeNbGeneralConstraints()
  {
    nbGeneralConstraints_ = 0;
    for(int i = 0; i<feetSupervisor_.getNbPreviewedSteps(); ++i)
    {
      nbGeneralConstraints_+=
          feetSupervisor_.getKinematicConvexPolygons()[i].getNbGeneralConstraints();
    }
  }


  void HumanoidFootConstraint::computeGeneralConstraintsMatrices(int sizeVec)
  {
    computeNbGeneralConstraints();

    int M = feetSupervisor_.getNbPreviewedSteps();

    A_.setZero(nbGeneralConstraints_,
               sizeVec);
    b_.setConstant(nbGeneralConstraints_, MAXIMUM_BOUND_VALUE);


    //Number of general constraints at current step
    int nbGeneralConstraintsAtCurrentStep(0);
    //Sum of general constraints numbers since first step
    int nbGeneralConstraintsSinceFirstStep(0);

    for(int i = 0; i<M; ++i)
    {
      nbGeneralConstraintsAtCurrentStep =
          feetSupervisor_.getKinematicConvexPolygons()[i].getNbGeneralConstraints();

      for(int j = 0; j<nbGeneralConstraintsAtCurrentStep; ++j)
      {
        // Filling matrix A and vector b to create a general constraint of the form
        // AX + b <=0
        A_(nbGeneralConstraintsSinceFirstStep + j, i) =
            feetSupervisor_.getKinematicConvexPolygons()[i]
            .getGeneralConstraintsMatrixCoefsForX()(j);

        A_(nbGeneralConstraintsSinceFirstStep + j, i + M) =
            feetSupervisor_.getKinematicConvexPolygons()[i]
            .getGeneralConstraintsMatrixCoefsForY()(j);

        b_(nbGeneralConstraintsSinceFirstStep + j) =
            feetSupervisor_.getKinematicConvexPolygons()[i]
            .getGeneralConstraintsConstantPart()(j);
      }

      nbGeneralConstraintsSinceFirstStep += nbGeneralConstraintsAtCurrentStep;
    }
  }

  void HumanoidFootConstraint::computeBoundsVectors(const VectorX& x0)
  {
    int M = feetSupervisor_.getNbPreviewedSteps();

    assert(x0.rows()==2*M);

    supBound_.setConstant(2*M, MAXIMUM_BOUND_VALUE);
    infBound_.setConstant(2*M, -MAXIMUM_BOUND_VALUE);

    for(int i = 0; i<M; ++i)
    {
      supBound_(i) = feetSupervisor_.getKinematicConvexPolygons()[i].getXSupBound();
      supBound_(i + M) = feetSupervisor_.getKinematicConvexPolygons()[i].getYSupBound();
      infBound_(i) = feetSupervisor_.getKinematicConvexPolygons()[i].getXInfBound();
      infBound_(i + M) = feetSupervisor_.getKinematicConvexPolygons()[i].getYInfBound();
    }

    // As we optimize a variation dX_ of the QP variable X_ (such that X_ = x0 + dX_),
    // the bound constraints need to be translated of x0 too.
    supBound_ -= x0;
    infBound_ -= x0;
  }

}
