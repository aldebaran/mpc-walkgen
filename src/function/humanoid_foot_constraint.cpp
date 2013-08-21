#include "humanoid_foot_constraint.h"

namespace MPCWalkgen
{
  HumanoidFootConstraint::HumanoidFootConstraint(const LIPModel& lipModel,
                                                 const HumanoidFootModel& leftFootModel,
                                                 const HumanoidFootModel& rightFootModel)
    :lipModel_(lipModel)
    ,leftFootModel_(leftFootModel)
    ,rightFootModel_(rightFootModel)
  {
    function_.fill(0);
    gradient_.setZero(1, 1);

    A_.setZero(1, 1);
    b_.fill(0);

    computeConstantPart();
  }

  HumanoidFootConstraint::~HumanoidFootConstraint()
  {}

  int HumanoidFootConstraint::getNbConstraints() const
  {
    assert(leftFootModel_.getNbPreviewedSteps() == rightFootModel_.getNbPreviewedSteps());
    assert(leftFootModel_.getKinematicHull().p.size()
           == rightFootModel_.getKinematicHull().p.size());


    return leftFootModel_.getKinematicHull().p.size()*
        leftFootModel_.getNbPreviewedSteps();
  }

  const VectorX& HumanoidFootConstraint::getFunction(const VectorX& x0)
  {
    assert(x0.rows()
           == 2*lipModel_.getNbSamples() + 2*leftFootModel_.getNbPreviewedSteps());
    assert(A_.rows()
           == leftFootModel_.getKinematicHull().p.size()*leftFootModel_.getNbPreviewedSteps());
    assert(A_.cols()
           == 2*lipModel_.getNbSamples() + 2*leftFootModel_.getNbPreviewedSteps());
    assert(A_.rows() == b_.rows());

    function_.noalias() = A_*x0;
    function_.noalias() -= b_;
    return function_;
  }

  const MatrixX& HumanoidFootConstraint::getGradient()
  {
    return gradient_;
  }

  void HumanoidFootConstraint::computeConstantPart()
  {
    computeconstraintMatrices();
    gradient_ = A_;
  }

  void HumanoidFootConstraint::computeconstraintMatrices()
  {
    assert(leftFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(rightFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(leftFootModel_.getNbPreviewedSteps() == rightFootModel_.getNbPreviewedSteps());
    assert(leftFootModel_.getKinematicHull().p.size()
           == rightFootModel_.getKinematicHull().p.size());

    int N = lipModel_.getNbSamples();
    int M = leftFootModel_.getNbPreviewedSteps();
    int hullSize = leftFootModel_.getKinematicHull().p.size();
    Vector3 p1;
    Vector3 p2;

    A_.setZero(M*hullSize, 2*N + 2*M);
    b_.resize(M*hullSize);

    for (int i=0; i<M; ++i)
    {
      for(int j=0; j<hullSize; ++j)
      {
      //TODO: delete isSupportFoot, replace by isInContact
      //      (It will be more convenient to do it at the next commit)
      if (leftFootModel_.isSupportFoot(i))
      {
        if (rightFootModel_.isSupportFoot(i))
        {
          //Double support. The foor constraint does not apply.
          //Left foot kinematic hull is the default choice
          //TODO: do something else?
          p1 = leftFootModel_.getKinematicHull().p[j];
          p2 = leftFootModel_.getKinematicHull().p[(j+1)%hullSize];
        }
        else
        {
          //Left foot is support foot
          p1 = leftFootModel_.getKinematicHull().p[j];
          p2 = leftFootModel_.getKinematicHull().p[(j+1)%hullSize];
        }
      }
      else if (rightFootModel_.isSupportFoot(i))
      {
        //Right foot is support foot
        p1 = rightFootModel_.getKinematicHull().p[j];
        p2 = rightFootModel_.getKinematicHull().p[(j+1)%hullSize];
      }
      else
      {
        std::cerr << "[Error] No foot in Contact \n";
        assert(false);
      }

        A_(i*hullSize + j, i) = p1(1) - p2(1);
        A_(i*hullSize + j, M + i) = p1(0) - p2(0);
        b_(i*hullSize + j) = (p2(0)-p1(0))*p1(1) - (p2(1)-p1(1))*p1(0);
      }
    }
  }
}
