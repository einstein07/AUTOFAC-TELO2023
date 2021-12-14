/*
 * @(#) CollisionAvoidanceHeuristic.h   1.0   Sep 28, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef EDQD_COLLISIONAVOIDANCEHEURISTIC_H_
#define EDQD_COLLISIONAVOIDANCEHEURISTIC_H_

#include "Heuristic.h"
#include "scenario/Scenario.h"
#include "Util.h"
#include "model/components/perceptive/ColorSensorModel.h"


namespace robogen{
//class EDQDRobot;
class CollisionAvoidanceHeuristic : public Heuristic{
	public:
		CollisionAvoidanceHeuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario);
		~CollisionAvoidanceHeuristic();
		virtual osg::Vec2d step();
};
}



#endif /* EDQD_COLLISIONAVOIDANCEHEURISTIC_H_ */
