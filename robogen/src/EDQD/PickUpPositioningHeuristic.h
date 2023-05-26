/*
 * @(#) PickUpPositioningHeuristic.h   1.0   Oct 25, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef EDQD_PICKUPPOSITIONINGHEURISTIC_H_
#define EDQD_PICKUPPOSITIONINGHEURISTIC_H_

#include "EDQD/Heuristic.h"
#include "scenario/Scenario.h"
#include "model/sensors/IrSensor.h"
#include "model/sensors/ColorSensor.h"
#include "model/sensors/TargetAreaDetector.h"

namespace robogen{
class AnchorPoint;
/**
 * \brief Class for describing a heuristic for positioning an agent before resource pick-up
 *
 * PickUpPositioningHeuristic inherits from Heuristic to describe a heuristic
 * for positioning a robot before picking up resources once agents are within
 * a certain distance to said resources located outside the gathering zone.
 *
 */
class PickUpPositioningHeuristic : public Heuristic {
	public:
		/**
		 * \brief Class constructor
		 * @param Robot The agent to utilize this heuristic
		 * @param Scenario The scenario in which this task is being carried out
		 * @return a heuristic object
		 */
		PickUpPositioningHeuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario);
		/**
		 * \brief Destructor
		 */
		~PickUpPositioningHeuristic();

		virtual osg::Vec2d step(boost::mutex& queueMutex);
		/**
		 * \brief Determines points on the resource on which agent can attach.
		 * @param BoxResource target resource
		 * @return osg::Vec3d attachment point in global coordinates.
		 */
		osg::Vec3d nextStep(boost::shared_ptr<BoxResource> resource);
		/**
		 * \brief Calculates the attachment point on the target resource
		 * @param BoxResource target resource
		 * @return osg::Vec3d coordinates (local relative to robot) of the attachment point
		 */
		osg::Vec3d calculateTargetPoint(boost::shared_ptr<BoxResource> resource);
		/**
		 * \brief Marks the resource that the agent using this heuristic is trying to pick-up
		 * @param BoxResource the resource to be collected
		 */
		void setResource(boost::shared_ptr<BoxResource> resource);

		/**
		 * \brief Returns the status of this heuristic
		 * @return bool true if this heuristic is active, otherwise false
		 */
		inline bool isActive(){return active_;}
		/**
		 * \brief Sets activity flag
		 * @param bool True if this heuristic is active, otherwise false
		 *
		 */
		inline void setActive(bool value){active_ = value;}


	private:
		/** Task environment */
		boost::shared_ptr<Environment> env;
		/** Gathering zone */
		osg::Vec3d targetAreaPosition_;
		/** Target attachment point */
		osg::Vec3d targetPoint;
		/** Target resource */
		boost::shared_ptr<BoxResource> resource_;
		/** Activity flag */
		bool active_;
};
}


#endif /* EDQD_PICKUPPOSITIONINGHEURISTIC_H_ */
