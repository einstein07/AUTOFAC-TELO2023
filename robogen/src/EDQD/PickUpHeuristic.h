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

namespace robogen{
//class EDQDRobot;
class PickUpHeuristic : public Heuristic {
	public:
		PickUpHeuristic(boost::shared_ptr<Robot> robot, osg::Vec2d targetAreaPosition);
		~PickUpHeuristic();
		virtual osg::Vec2d step(boost::shared_ptr<Environment>& env, boost::shared_ptr<Scenario> scenario);

	private:
		osg::Vec2d targetAreaPosition_;
		bool ENABLE_PICKUP_POSITIONING = false;
};
}


#endif /* EDQD_PICKUPHEURISTIC_H_ */
