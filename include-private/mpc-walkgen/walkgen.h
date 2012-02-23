#ifndef WALKGEN
#define WALKGEN

////////////////////////////////////////////////////////////////////////////////
///
///\file	walkgen.h
///\brief	Main program of the MPC
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author	Keith François
///\version	1.0
///\date	05/01/12
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/walkgen-abstract.h>
#include <mpc-walkgen/types.h>

namespace MPCWalkgen{
	class LSSOLSolver;
	class QPGenerator;
	class QPPreview;
	class Interpolation;
	class RigidBodySystem;
	class OrientationsPreview;
	class MPCDebug;

	class Walkgen :
		public WalkgenAbstract
	{

		public:
			Walkgen();
			~Walkgen();

			virtual void init(const RobotData &robotData, const MPCData &mpcData);

			virtual const MPCSolution &online(double time, bool previewBodiesNextState=true);

		    /// \name Accessors and mutators
		    /// \{
			void reference(double dx, double dy, double dyaw);
		    /// \}

		  public:
		    /// \name accessors relative to the state of the robot.
		    /// \{
			virtual inline double comHeight() const;
			virtual inline void comHeight (double d);

			virtual inline double freeFlyingFootMaxHeight() const;
			virtual inline void freeFlyingFootMaxHeight(double d);

			virtual inline const SupportState &currentSupportState() const;
			virtual inline void currentSupportState(const SupportState &newSupportState){
				newCurrentSupport_=newSupportState;
				isNewCurrentSupport_=true;
			}

			virtual const BodyState & bodyState(BodyType body)const;
			virtual void bodyState(BodyType body, const BodyState & state);
			/// \}


			/// \name Accessors relative to the solver, modifiable on line
			/// \{
		  public:
			virtual inline double stepPeriod()const
			{ return generalData_.stepPeriod; }
			virtual inline void stepPeriod(double d)
			{ generalData_.stepPeriod = d; }

			virtual inline double DSPeriod()const
			{ return generalData_.DSPeriod; }
			virtual inline void DSPeriod(double d)
			{ generalData_.DSPeriod = d; }

			virtual inline double DSSSPeriod()const
			{ return generalData_.DSSSPeriod; }
			virtual inline void DSSSPeriod(double d)
			{ generalData_.DSSSPeriod = d; }

			virtual inline int nbStepSSDS()const
			{ return generalData_.nbStepSSDS; }
			virtual inline void nbStepSSDS(int d)
			{ generalData_.nbStepSSDS = d; }
			/// \}

		  public:
			/// \name accessors relative to the solver, costly when modified on line
		    /// \{
			virtual inline double QPSamplingPeriod()const
			{return generalData_.QPSamplingPeriod;}
			virtual void QPSamplingPeriod(double d);

			virtual inline double mpcSamplingPeriod()const
			{return generalData_.MPCSamplingPeriod;}
			virtual void mpcSamplingPeriod(double d);

			virtual inline double simSamplingPeriod()const
			{return generalData_.simSamplingPeriod;}
			virtual void simSamplingPeriod(double d);

			virtual inline int QPNbSamplings()const
			{return generalData_.nbSamplesQP;}
			virtual void QPNbSamplings(int d);
		    /// \}

		  private:
			MPCData generalData_;

		    LSSOLSolver * solver_;
			QPGenerator * generator_;
			QPPreview * preview_;
			Interpolation * interpolation_;
			RigidBodySystem * robot_;

			OrientationsPreview * orientPrw_;

			MPCSolution solution_;
			VelReference velRef_;
			/// \brief The new value of reference velocity, updated with in online method
			VelReference newVelRef_;
			QPPonderation ponderation_;
			/// \brief The new value of current support state, updated with in online method
			SupportState newCurrentSupport_;
			bool isNewCurrentSupport_;

			MPCDebug* debug_;
			bool enableDisplay_;

		    /// \brief Time at which the problem should be updated
			double upperTimeLimitToUpdate_;
			double upperTimeLimitToFeedback_;

			/// \brief Synchronised time with QP sampling
			double currentTime_;

	};
}

/*! \fn MPCWalkgen::Walkgen::Walkgen(const MPCData & generalData, const RobotData & robotData)
* \brief Constructor
* \param generalData options of the mpc
* \param robotData   caracteritics of the robot
*/

/*! \fn MPCWalkgen::Walkgen::init(const Eigen::Vector3d & leftFootPosition, const Eigen::Vector3d & rightFootPosition)
* \brief Initialize the algorithm
* \param leftFootPosition position of the left foot in absolute coordinates
* \param rightFootPosition position of the right foot in absolute coordinates
*/

/*! \fn const MPCSolution & MPCWalkgen::Walkgen::online(double time)
* \brief Compute one iteration of the algorithm and return the solution
*/

#endif //WALKGEN
