/*
 * @(#) PickUpHeuristic.h   1.0   Sep 28, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef EDQD_PICKUPHEURISTIC_H_
#define EDQD_PICKUPHEURISTIC_H_

#include "EDQD/Heuristic.h"
#include "scenario/Scenario.h"
#include "model/sensors/IrSensor.h"
#include "model/sensors/ColorSensor.h"
#include "model/sensors/TargetAreaDetector.h"
#include "EDQD/PickUpPositioningHeuristic.h"
namespace robogen{
/**
 * \brief Class for describing a resource pick-up heuristic
 *
 * PickUpHeuristic inherits from Heuristic to describe a heuristic
 * for picking up resources once agents are within
 * a certain distance to said resources located outside the gathering zone.
 *
 */
class PickUpHeuristic : public Heuristic {
	public:
		/**
		 * \brief Class constructor
		 *
		 * @param Robot The agent to utilize this heuristic
		 * @param Scenario The scenario in which this task is being carried out
		 * @return a heuristic object
		 */
		PickUpHeuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario);
		/**
		 * \brief Destructor
		 */
		~PickUpHeuristic();
		virtual osg::Vec2d step(boost::mutex& queueMutex);

	private:
		/** Current task environment */
		boost::shared_ptr<Environment> env;
		/** Coordinates of the gathering zone */
		osg::Vec3d targetAreaPosition_;
		/** Flag if pick-up positioning should be enabled */
		bool ENABLE_PICKUP_POSITIONING = true;
		/** Pickup positioning heuristic */
		boost::shared_ptr<PickUpPositioningHeuristic> heuristicpp_;
};
}


#endif /* EDQD_PICKUPHEURISTIC_H_ */
