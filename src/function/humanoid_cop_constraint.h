////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_cop_constraint.h
///\brief Implement the CoP constraint
///\author de Gourcuff Martin
///\date 12/07/13
///
////////////////////////////////////////////////////////////////////////////////

#ifndef MPC_WALKGEN_HUMANOID_COP_CONSTRAINT_H
#define MPC_WALKGEN_HUMANOID_COP_CONSTRAINT_H

#include "../type.h"
#include "../model/lip_model.h"
#include "../humanoid_feet_supervisor.h"

namespace MPCWalkgen
{
  class HumanoidCopConstraint
  {
    public:
      HumanoidCopConstraint(const LIPModel& lipModel,
                            const HumanoidFeetSupervisor& feetSupervisor);
      ~HumanoidCopConstraint();

      unsigned int getNbConstraints();
      const VectorX& getFunction(const VectorX &x0);
      const MatrixX& getGradient(unsigned int sizeVec);
      const VectorX& getSupBounds();
      const VectorX& getInfBounds();

      void computeConstantPart();

    private:
      /// \brief Compute the number of general constraints over the whole
      ///        QP preview window
      void xComputeNbGeneralConstraints();
      /// \brief Compute general constraints Matrices A and b
      ///        for constraint of the form A*X +b <= 0
      void xComputeGeneralConstraintsMatrices(unsigned int sizeVec);
      /// \brief Compute bounds vectors infBound_ and supBound_
      ///        for constraint of the form infBound_ <= X <= supBound_
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
#endif // MPC_WALKGEN_HUMANOID_COP_CONSTRAINT_H
