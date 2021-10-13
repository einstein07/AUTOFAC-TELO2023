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
		CollisionAvoidanceHeuristic(boost::shared_ptr<Robot> robot);
		~CollisionAvoidanceHeuristic();
		virtual osg::Vec2d step(boost::shared_ptr<Environment>& env, boost::shared_ptr<Scenario> scenario);
};
}



#endif /* EDQD_COLLISIONAVOIDANCEHEURISTIC_H_ */
