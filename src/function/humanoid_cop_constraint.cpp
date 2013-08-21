#include "humanoid_cop_constraint.h"

namespace MPCWalkgen
{
  HumanoidCopConstraint::HumanoidCopConstraint(const LIPModel& lipModel,
                                               const HumanoidFootModel& leftFootModel,
                                               const HumanoidFootModel& rightFootModel)
    :lipModel_(lipModel)
    ,leftFootModel_(leftFootModel)
    ,rightFootModel_(rightFootModel)
    ,function_(1)
  {
    assert(leftFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(rightFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(leftFootModel_.getCopDSHull().p.size()==4);
    assert(leftFootModel_.getCopSSHull().p.size()==4);

    function_.fill(0);
    computeConstantPart();
  }

  HumanoidCopConstraint::~HumanoidCopConstraint()
  {}

  int HumanoidCopConstraint::getNbConstraints() const
  {
    assert(leftFootModel_.getNbSamples() == rightFootModel_.getNbSamples());
    assert(leftFootModel_.getCopDSHull().p.size() == 4);
    assert(leftFootModel_.getCopDSHull().p.size()
           == rightFootModel_.getCopDSHull().p.size());
    assert(leftFootModel_.getCopSSHull().p.size()
           == rightFootModel_.getCopSSHull().p.size());
    assert(leftFootModel_.getCopDSHull().p.size()
           == leftFootModel_.getCopSSHull().p.size() );

    return 4*lipModel_.getNbSamples();
  }

  const VectorX& HumanoidCopConstraint::getFunction()
  {
    assert(leftFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(rightFootModel_.getNbSamples() == lipModel_.getNbSamples());
    assert(leftFootModel_.getCopDSHull().p.size()==4);
    assert(rightFootModel_.getCopSSHull().p.size()==4);

    int N = lipModel_.getNbSamples();
    function_.resize(4*N);

    for (int i = 0; i<N; ++i)
    {
      if(leftFootModel_.isInContact(i))
      {
        if(rightFootModel_.isInContact(i))
        {
          //Double support
          //TODO: merge left and right support hull

          function_[i] = dsBoundsMin_(0);
          function_[i + N] = dsBoundsMin_(1);
          function_[i + 2*N] = dsBoundsMax_(0);
          function_[i + 3*N] = dsBoundsMax_(1);
        }
        else{
          //Left foot is support foot

          function_[i] = ssLeftFootBoundsMin_(0);
          function_[i + N] = ssLeftFootBoundsMin_(1);
          function_[i + 2*N] = ssLeftFootBoundsMax_(0);
          function_[i + 3*N] = ssLeftFootBoundsMax_(1);
        }
      }
      else if(rightFootModel_.isInContact(i))
      {
        function_[i] = ssRightFootBoundsMin_(0);
        function_[i + N] = ssRightFootBoundsMin_(1);
        function_[i + 2*N] = ssRightFootBoundsMax_(0);
        function_[i + 3*N] = ssRightFootBoundsMax_(1);
      }
      else
      {
        std::cerr << "[Error] No foot in Contact \n";
        assert(false);
      }
    }

    return function_;
  }

  void HumanoidCopConstraint::computeConstantPart()
  {
    ssLeftFootBoundsMin_ = leftFootModel_.getCopSSHull().getVectorMin();
    ssLeftFootBoundsMax_ = leftFootModel_.getCopSSHull().getVectorMax();

    ssRightFootBoundsMin_ = rightFootModel_.getCopSSHull().getVectorMin();
    ssRightFootBoundsMax_ = rightFootModel_.getCopSSHull().getVectorMax();

    dsBoundsMin_ = leftFootModel_.getCopDSHull().getVectorMin();
    dsBoundsMax_ = leftFootModel_.getCopDSHull().getVectorMax();
  }
}
