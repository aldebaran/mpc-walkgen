////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_foot_constraint.h
///\brief Implement the foot constraints
///\author de Gourcuff Martin
///\date 12/07/13
///
////////////////////////////////////////////////////////////////////////////////

#ifndef MPC_WALKGEN_HUMANOID_FOOT_CONSTRAINT_H
#define MPC_WALKGEN_HUMANOID_FOOT_CONSTRAINT_H

#include "../type.h"
#include "../model/lip_model.h"
#include "../humanoid_feet_supervisor.h"

namespace MPCWalkgen
{
  class HumanoidFootConstraint
  {
    public:
      HumanoidFootConstraint(const LIPModel& lipModel,
                             const HumanoidFeetSupervisor& feetSupervisor);
      ~HumanoidFootConstraint();

      unsigned int getNbConstraints();
      const VectorX& getFunction(const VectorX &x0);
      const MatrixX& getGradient(unsigned int sizeVec);
      const VectorX& getSupBounds();
      const VectorX& getInfBounds();

    private:
      /// \brief Compute the number of general constraints over the whole
      ///        QP preview window
      void xComputeNbGeneralConstraints();
      /// \brief Compute general constraints Matrices A and b
      ///        such that A*X + b <= 0
      void xComputeGeneralConstraintsMatrices(unsigned int sizeVec);
      /// \brief Compute bounds vectors infBound_ and supBound_
      ///        such that infBound_ <= X <= supBound_
      void xComputeBoundsVectors();

      const LIPModel& lipModel_;
      const HumanoidFeetSupervisor& feetSupervisor_;

      unsigned int nbGeneralConstraints_;

      VectorX function_;
      MatrixX gradient_;

      VectorX supBound_;
      VectorX infBound_;

      MatrixX A_;
      VectorX b_;
  };
}
#endif // MPC_WALKGEN_HUMANOID_FOOT_CONSTRAINT_H
