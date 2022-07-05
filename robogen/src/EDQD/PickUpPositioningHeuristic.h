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
class PickUpPositioningHeuristic : public Heuristic {
	public:
		PickUpPositioningHeuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario);
		~PickUpPositioningHeuristic();

		virtual osg::Vec2d step();
		osg::Vec3d nextStep(boost::shared_ptr<BoxResource> resource);
		osg::Vec3d calculateTargetPoint(boost::shared_ptr<BoxResource> resource);
		void setResource(boost::shared_ptr<BoxResource> resource);

		inline bool isActive(){return active_;}
		inline void setActive(bool value){active_ = value;}


	private:
		boost::shared_ptr<Environment> env;
		osg::Vec3d targetAreaPosition_;
		osg::Vec3d targetPoint;
		boost::shared_ptr<BoxResource> resource_;
		bool active_;
};
}


#endif /* EDQD_PICKUPPOSITIONINGHEURISTIC_H_ */
