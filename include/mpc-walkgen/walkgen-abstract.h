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

	  WalkgenAbstract(
	    		const FootData & leftFoot, const FootData & rightFoot,
	    		const HipYawData & leftHipYaw, const HipYawData & rightHipYaw,
	    		double robotMass, double comHeight);

	  virtual ~WalkgenAbstract() =0;


	    /*!  \brief Set the ModulationSupportCoefficient.*/
	    ///virtual void SetModulationSupportCoefficient(double) =0;

	    /*! \name Call method to handle on-line generation of ZMP reference trajectory.
	      @{*/

	    /*! Methods for on-line generation.
	      @param[in] InitLeftFootAbsolutePosition: The initial position of the left foot.
	      @param[in] InitRightFootAbsolutePosition: The initial position of the right foot.
	     */

	    virtual void init(
	        const Eigen::Vector3d & initLeftFootAbsolutePosition,
	        const Eigen::Vector3d & initRightFootAbsolutePosition,
			const RobotData & robotData, const MPCData & mpcData) =0;


	    /// \brief Update the stacks on-line
	    /* ! \brief Method to change to update on line the queues necessary of the system.
	       @param[in] time : Current time.

	       @return The associated solution
	       If solution.newTraj is true, the method has succeeded.
	     */
	    virtual const MPCSolution & online(double time, bool previewBodiesNextState=true) =0;

	    /// \name Accessors and mutators
	    /// \{
	    /// \brief Set the reference (velocity only as for now) through the Interface (slow)
	    virtual void reference(double dx, double dy, double dyaw) =0;


		//accessors relative to the state of the robot.
	  public:
	    /* ! \brief Returns the Com Height. */
		virtual double comHeight()const=0;
	    /* ! \brief Sets the Com Height. */
		virtual void comHeight(double d)=0;

		virtual double freeFlyingFootMaxHeight() const = 0;
		virtual void freeFlyingFootMaxHeight(double d)=0;

		virtual const SupportState & currentSupportState() const = 0;
		virtual void currentSupportState(const SupportState & newSupportState)=0;

		virtual const BodyState & bodyState(BodyType body)const=0;
		virtual void bodyState(BodyType body, const BodyState & state)=0;

	  public:
		//accessors relative to the solver, modifiable on line
		virtual double stepPeriod()const=0;
		virtual void stepPeriod(double d)=0;

		virtual double DSPeriod()const=0;
		virtual void DSPeriod(double d)=0;

		virtual double DSSSPeriod()const=0;
		virtual void DSSSPeriod(double d)=0;

		virtual int nbStepSSDS()const=0;
		virtual void nbStepSSDS(int d)=0;

		//accessors relative to the solver, should not be modified on line
	  public:
		virtual double QPSamplingPeriod()const=0;
		virtual void QPSamplingPeriod(double d)=0;

		virtual double mpcSamplingPeriod()const=0;
		virtual void mpcSamplingPeriod(double d)=0;

		virtual double simSamplingPeriod()const=0;
		virtual void simSamplingPeriod(double d)=0;

		virtual int QPNbSamplings()const=0;
		virtual void QPNbSamplings(int d)=0;
  };

  /*! Factory of Pattern generator interface. */
  MPC_WALK_GEN_EXPORT WalkgenAbstract * mpcFactory(
	const FootData & leftFoot, const FootData & rightFoot,
	const HipYawData & leftHipYaw, const HipYawData & rightHipYaw,
	double robotMass, double comHeight,
	const std::string & qpParams =""
  );
}

#endif /* _WALKGEN_ABSTRACT_H_ */
