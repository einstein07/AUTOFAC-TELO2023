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

class PickUpPositioningHeuristic : public Heuristic {
	public:
		PickUpPositioningHeuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario);
		~PickUpPositioningHeuristic();

		virtual osg::Vec2d step();
		osg::Vec2d nextStep(BoxResource resource);
		osg::Vec2d calculateTargetPoint(BoxResource resource);


	private:
		boost::shared_ptr<Environment> env;
		osg::Vec3d targetAreaPosition_;
		bool ENABLE_PICKUP_POSITIONING = false;
};
}


#endif /* EDQD_PICKUPPOSITIONINGHEURISTIC_H_ */
