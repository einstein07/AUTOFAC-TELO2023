/*
 * @(#) DropOffHeuristic.h   1.0   Nov 04, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef EDQD_DROPOFFHEURISTIC_H_
#define EDQD_DROPOFFHEURISTIC_H_

#include "EDQD/Heuristic.h"
#include "scenario/Scenario.h"
#include "model/sensors/IrSensor.h"
#include "model/sensors/ColorSensor.h"
#include "model/sensors/TargetAreaDetector.h"
#include "EDQD/PickUpPositioningHeuristic.h"
namespace robogen{
/**
 * \brief Class for describing a resource drop-off heuristic
 *
 * DropOffHeuristic inherits from Heuristic to describe a heuristic
 * for dropping off collected resources once agents are within the gathering zone.
 *
 */
class DropOffHeuristic : public Heuristic {
	public:
		/**
		 * Class constructor
		 * @param Robot The agent to utilize this heuristic
		 * @param Scenario The scenario in which this task is being carried out
		 * @return a heuristic object
		 */
		DropOffHeuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario);
		/**
		 * Destructor
		 */
		~DropOffHeuristic();
		virtual osg::Vec2d step(boost::mutex& queueMutex);

	private:
		/** Task environment */
		boost::shared_ptr<Environment> env;
		/** Coordinates of gathering zone */
		osg::Vec3d targetAreaPosition_;

};
}


#endif /* EDQD_DROPOFFHEURISTIC_H_ */
