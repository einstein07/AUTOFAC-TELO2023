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
//class EDQDRobot;
class PickUpHeuristic : public Heuristic {
	public:
		PickUpHeuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario);
		~PickUpHeuristic();
		virtual osg::Vec2d step();

	private:
		boost::shared_ptr<Environment> env;
		osg::Vec3d targetAreaPosition_;
		bool ENABLE_PICKUP_POSITIONING = false;
		boost::shared_ptr<PickUpPositioningHeuristic> heuristicpp_;
};
}


#endif /* EDQD_PICKUPHEURISTIC_H_ */
