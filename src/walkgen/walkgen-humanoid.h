#ifndef WALKGENHUMANOID
#define WALKGENHUMANOID

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

#include <mpc-walkgen/walkgen-abstract-humanoid.h>
#include "types.h"

namespace MPCWalkgen{
	class QPSolver;
	class QPGenerator;
	class QPPreview;
	class Interpolation;
	class RigidBodySystem;
	class OrientationsPreview;
	class MPCDebug;
	class Perturbation;

	class WalkgenHumanoid :
		public WalkgenAbstractHumanoid
	{

		public:
			WalkgenHumanoid(Solver solvertype);
			~WalkgenHumanoid();

			virtual void init(const RobotData &robotData, const MPCData &mpcData);

			virtual const MPCSolution &online(double time, bool previewBodiesNextState=true);

		    /// \name Accessors and mutators
		    /// \{
			void reference(double dx, double dy, double dyaw);
			void reference(Eigen::VectorXd dx, Eigen::VectorXd dy, Eigen::VectorXd dyaw);
		    /// \}

		  public:
		    /// \name accessors relative to the state of the robot.
		    /// \{
			inline const RigidBodySystem *robot() const
			{return robot_;}

			virtual double comHeight() const;
			virtual void comHeight (double d);

			virtual double freeFlyingFootMaxHeight() const;
			virtual void freeFlyingFootMaxHeight(double d);

			virtual const SupportState &currentSupportState() const;
			virtual inline void currentSupportState(const SupportState &newSupportState){
				newCurrentSupport_=newSupportState;
				isNewCurrentSupport_=true;
			}

			virtual const BodyState & bodyState(BodyType body)const;
			virtual void bodyState(BodyType body, const BodyState & state);
			/// \}


			/// \name Accessors relative to the support logic
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
			/// \name accessors relative to the solver
		    /// \{
			virtual inline double QPSamplingPeriod()const
			{return generalData_.QPSamplingPeriod;}
			virtual void QPSamplingPeriod(double d);

			virtual inline double mpcSamplingPeriod()const
			{return generalData_.MPCSamplingPeriod;}
			virtual void mpcSamplingPeriod(double d);

			virtual inline double actuationSamplingPeriod()const
			{return generalData_.actuationSamplingPeriod;}
			virtual void actuationSamplingPeriod(double period)
			{generalData_.actuationSamplingPeriod = period;}

			virtual inline int QPNbSamplings()const
			{return generalData_.nbSamplesQP;}
			virtual void QPNbSamplings(int d);
		    /// \}

		  public:
			/// \name accessors relative to perturbation
		    /// \{
			virtual void ApplyPerturbationForce(Axis axis, BodyType body, double f);
			virtual void ApplyPerturbationAcc(Axis axis, BodyType body, double acc);
			virtual void ApplyPerturbationVel(Axis axis, BodyType body, double vel);
		    /// \}

		  private:
			MPCData generalData_;

			QPSolver * solver_;
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

			Perturbation * perturbation_;

			MPCDebug* debug_;
			bool enableDisplay_;

		    /// \brief Time at which the problem should be updated
			double upperTimeLimitToUpdate_;
			double upperTimeLimitToFeedback_;

			/// \brief Synchronised time with QP sampling
			double currentTime_;

	};
}



#endif //WALKGENHUMANOID
