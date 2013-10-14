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
          feetSupervisor_.getKinematicConvexPolygon(i).getNbGeneralConstraints();
    }
  }


  void HumanoidFootConstraint::computeGeneralConstraintsMatrices(int sizeVec)
  {
    computeNbGeneralConstraints();

    int N = feetSupervisor_.getNbSamples();
    int M = feetSupervisor_.getNbPreviewedSteps();

    A_.setZero(nbGeneralConstraints_,sizeVec);
    b_.setConstant(nbGeneralConstraints_, MAXIMUM_BOUND_VALUE);


    //Number of general constraints at current step
    int nbGeneralConstraintsAtCurrentStep(0);
    //Sum of general constraints numbers since first step
    int nbGeneralConstraintsSinceFirstStep(0);

    for(int i = 0; i<M; ++i)
    {
      const ConvexPolygon& cp = feetSupervisor_.getKinematicConvexPolygon(i);

      nbGeneralConstraintsAtCurrentStep = cp.getNbGeneralConstraints();

      for(int j = 0; j<nbGeneralConstraintsAtCurrentStep; ++j)
      {
        // Filling matrix A and vector b to create a general constraint of the form
        // AX + b <=0
        A_(nbGeneralConstraintsSinceFirstStep + j, 2*N + i) =
            cp.getGeneralConstraintsMatrixCoefsForX()(j);

        A_(nbGeneralConstraintsSinceFirstStep + j, 2*N + M + i) =
            cp.getGeneralConstraintsMatrixCoefsForY()(j);

        b_(nbGeneralConstraintsSinceFirstStep + j) =
            cp.getGeneralConstraintsConstantPart()(j);

        // This if-else statement adds the required terms to matrices A and b so that
        // constraints are expressed in world frame
        if(i==0)
        {
          b_(nbGeneralConstraintsSinceFirstStep + j) -=
              cp.getGeneralConstraintsMatrixCoefsForX()(j)
              *feetSupervisor_.getSupportFootStateX()(0)
              + cp.getGeneralConstraintsMatrixCoefsForY()(j)
              *feetSupervisor_.getSupportFootStateY()(0);
        }
        else
        {
          A_(nbGeneralConstraintsSinceFirstStep + j, 2*N + i - 1) -=
              cp.getGeneralConstraintsMatrixCoefsForX()(j);

          A_(nbGeneralConstraintsSinceFirstStep + j, 2*N + i - 1 + M) -=
              cp.getGeneralConstraintsMatrixCoefsForY()(j);
        }
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
      const ConvexPolygon& cp = feetSupervisor_.getKinematicConvexPolygon(i);

      supBound_(i) = cp.getXSupBound();
      supBound_(i + M) = cp.getYSupBound();
      infBound_(i) = cp.getXInfBound();
      infBound_(i + M) = cp.getYInfBound();

      // This if-else statement adds the required terms to vectors supBound_ and infBound_
      // so that constraints are expressed in world frame
      if(i==0)
      {
        supBound_(i) += feetSupervisor_.getSupportFootStateX()(0);
        supBound_(i + M) += feetSupervisor_.getSupportFootStateY()(0);
        infBound_(i) += feetSupervisor_.getSupportFootStateX()(0);
        infBound_(i + M) += feetSupervisor_.getSupportFootStateY()(0);
      }
      else
      {
        supBound_(i) += x0(i - 1);
        supBound_(i + M) += x0(M + i - 1);
        infBound_(i) += x0(i - 1);
        infBound_(i + M) += x0(M + i - 1);
      }
    }

    // As we optimize a variation dX_ of the QP variable X_ (such that X_ = x0 + dX_),
    // the bound constraints need to be translated of x0 too.
    supBound_ -= x0;
    infBound_ -= x0;
  }

}
