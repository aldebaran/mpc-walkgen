////////////////////////////////////////////////////////////////////////////////
///
///\file	walkgen-abstract.h
///\brief	Abstract class to instanciate Walkgen algorithm
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author	Keith François
///\version	1.0
///\date	05/01/12
///
////////////////////////////////////////////////////////////////////////////////


#ifndef _WALKGEN_ABSTRACT_H_
#define _WALKGEN_ABSTRACT_H_

#include <mpc-walkgen/sharedpgtypes.h>

#include <Eigen/Core>
#include <cstring>
#include <deque>


namespace MPCWalkgen
{
  class  WalkgenAbstract
  {
	    //
	    // Public methods:
	    //
	  public:

	  WalkgenAbstract();

	  virtual ~WalkgenAbstract() =0;

	    /// \brief Initialize the system
	    /// \param[in] robotData: data relative to the robot
	    /// \param[in] mpcData: data relative to the qp solver
	    virtual void init(const RobotData &robotData, const MPCData &mpcData) = 0;


	    /// \brief Call method to handle on-line generation of ZMP reference trajectory.
	    /// \param[in] time : Current time.
	    /// \param[in] previewBodiesNextState

	    /// \return The associated solution
	    ///   If solution.newTraj is true, the method has succeeded.
	    virtual const MPCSolution & online(double time, bool previewBodiesNextState = true) = 0;

	    /// \name Accessors and mutators
	    /// \{
	    /// \brief Set the reference (velocity only as for now)
	    virtual void reference(double dx, double dy, double dyaw) = 0;
	    /// \}

	    /// \name accessors relative to the state of the robot.
	    /// \{
		virtual double comHeight()const=0;
		virtual void comHeight(double d)=0;

		virtual double freeFlyingFootMaxHeight() const = 0;
		virtual void freeFlyingFootMaxHeight(double d)=0;

		virtual const SupportState & currentSupportState() const = 0;
		virtual void currentSupportState(const SupportState & newSupportState)=0;

		virtual const BodyState & bodyState(BodyType body)const=0;
		virtual void bodyState(BodyType body, const BodyState & state)=0;
	    /// \}

	  public:
		/// \name accessors relative to the solver, modifiable on line
	    /// \{
		virtual double stepPeriod()const=0;
		virtual void stepPeriod(double d)=0;

		virtual double DSPeriod()const=0;
		virtual void DSPeriod(double d)=0;

		virtual double DSSSPeriod()const=0;
		virtual void DSSSPeriod(double d)=0;

		virtual int nbStepSSDS()const=0;
		virtual void nbStepSSDS(int d)=0;
	    /// \}

	  public:
		/// \name accessors relative to the solver, costly when modified on line
	    /// \{
		virtual double QPSamplingPeriod()const=0;
		virtual void QPSamplingPeriod(double d)=0;

		virtual double mpcSamplingPeriod()const=0;
		virtual void mpcSamplingPeriod(double d)=0;

		virtual double actuationSamplingPeriod()const=0;
		virtual void actuationSamplingPeriod(double d)=0;

		virtual int QPNbSamplings()const=0;
		virtual void QPNbSamplings(int d)=0;
	    /// \}
  };

  /*! Factory of Pattern generator interface. */
  MPC_WALK_GEN_EXPORT WalkgenAbstract * mpcFactory();
}

#endif /* _WALKGEN_ABSTRACT_H_ */
