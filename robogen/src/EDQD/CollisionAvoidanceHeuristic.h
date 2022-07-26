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
#include "model/components/perceptive/SensorMorphologyModel.h"
#include "model/components/perceptive/TargetAreaDetectorModel.h"


namespace robogen{
//class EDQDRobot;
class CollisionAvoidanceHeuristic : public Heuristic{
	private:
		bool staticObject;
		int avoidCounter;
		osg::Vec2d signal;
	public:
		CollisionAvoidanceHeuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario);
		~CollisionAvoidanceHeuristic();
		virtual osg::Vec2d step(boost::mutex& queueMutex);
		osg::Vec2d stepStatic();
		void decrement();
		inline bool isStaticObject(){return staticObject;}
		inline void setStaticObject(bool value){staticObject = value;}

		inline void resetCounter(int value){avoidCounter = value;}
		inline int getCounter(){return avoidCounter;}
		inline osg::Vec2d getSignal(){return signal;}
};
}



#endif /* EDQD_COLLISIONAVOIDANCEHEURISTIC_H_ */
