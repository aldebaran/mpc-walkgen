////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_cop_constraint.h
///\brief Implement the CoP constraint
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#pragma once
#ifndef MPC_WALKGEN_FUNCTION_HUMANOID_COP_CONSTRAINT_H
#define MPC_WALKGEN_FUNCTION_HUMANOID_COP_CONSTRAINT_H

#include <mpc-walkgen/api.h>
#include <mpc-walkgen/type.h>
#include <mpc-walkgen/model/lip_model.h>
#include <mpc-walkgen/humanoid_feet_supervisor.h>

#ifdef _MSC_VER
# pragma warning( push )
// C4251: class needs to have DLL interface
# pragma warning( disable: 4251 )
#endif

namespace MPCWalkgen
{
  template <typename Scalar>
  class MPC_WALKGEN_API HumanoidCopConstraint
  {
    TEMPLATE_TYPEDEF(Scalar)

    public:
      HumanoidCopConstraint(const LIPModel<Scalar>& lipModel,
                            const HumanoidFeetSupervisor<Scalar>& feetSupervisor);
      ~HumanoidCopConstraint();

      int getNbConstraints();
      const VectorX& getFunction(const VectorX &x0);
      const MatrixX& getGradient(int sizeVec);
      const VectorX& getSupBounds(const VectorX& x0);
      const VectorX& getInfBounds(const VectorX& x0);

      void computeConstantPart();

    private:
      /// \brief Compute the number of general constraints over the whole
      ///        QP preview window
      void computeNbGeneralConstraints();
      /// \brief Compute general constraints Matrices A and b
      ///        for constraint of the form A*X +b <= 0
      void computeGeneralConstraintsMatrices(int sizeVec);
      /// \brief Compute bounds vectors infBound_ and supBound_
      ///        for constraint of the form infBound_ <= X <= supBound_
      void computeBoundsVectors(const VectorX &x0);

    private:
      const LIPModel<Scalar>& lipModel_;
      const HumanoidFeetSupervisor<Scalar>& feetSupervisor_;

      int nbGeneralConstraints_;

      VectorX function_;
      MatrixX gradient_;

      VectorX supBound_;
      VectorX infBound_;

      MatrixX A_;
      VectorX b_;
  };
}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

#endif
