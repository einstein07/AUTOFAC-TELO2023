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
class DropOffHeuristic : public Heuristic {
	public:
		DropOffHeuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario);
		~DropOffHeuristic();
		virtual osg::Vec2d step();

	private:
		boost::shared_ptr<Environment> env;
		osg::Vec3d targetAreaPosition_;

};
}


#endif /* EDQD_DROPOFFHEURISTIC_H_ */
