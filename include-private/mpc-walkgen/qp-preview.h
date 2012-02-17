#ifndef QP_PREVIEW
#define QP_PREVIEW

////////////////////////////////////////////////////////////////////////////////
///
///\file	qp-preview.h
///\brief	A class to compute preview support state and selection matrices
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author	Keith François
///\version	1.0
///\date	05/01/12
///
////////////////////////////////////////////////////////////////////////////////


#include <mpc-walkgen/rigid-body-system.h>
#include <mpc-walkgen/state-solver.h>
#include <mpc-walkgen/types.h>

#include <Eigen/Dense>


namespace MPCWalkgen{

	class QPPreview{
		private:
			static const double EPS_;

		public:
			QPPreview(VelReference * velRef, RigidBodySystem * robot, const MPCData * generalData);

			~QPPreview();

			void previewSupportStates(const double currentTime,
					const double FirstIterationDynamicsDuration, MPCSolution & result, SupportState & CurrentSupport);

			void computeRotationMatrix(MPCSolution & result);

			inline SelectionMatrices & selectionMatrices(){return selectionMatrices_;}

			inline const Eigen::MatrixXd & rotationMatrix() const{return rotationMatrix_;}

			inline const Eigen::MatrixXd & rotationMatrix2() const{return rotationMatrix2_;}

		private:

			void buildSelectionMatrices(MPCSolution & result);



		private:
			RigidBodySystem * robot_;
			const MPCData * generalData_;
			StateSolver * statesolver_;

			SelectionMatrices selectionMatrices_;
			Eigen::MatrixXd rotationMatrix_;
			Eigen::MatrixXd rotationMatrix2_;

	};
}

/*! \fn MPCWalkgen::QPPreview::QPPreview(VelReference * velRef, RigidBodySystem * robot, GeneralData * generalData)
* \brief Constructor
*/

/*! \fn void MPCWalkgen::QPPreview::previewSupportStates(const double currentTime,
					const double FirstIterationDynamicsDuration, MPCSolution & result, SupportState & CurrentSupport)
* \brief Preview support states for the defined horizon
* \param currentTime current time (synchronized with QP sampling time)
* \param FirstIterationDynamicsDuration Duration of the first iteration
*/

/*! \fn SelectionMatrix & MPCWalkgen::QPPreview::state()
* \brief Return computed selection matrices
*/

#endif //QP_PREVIEW
