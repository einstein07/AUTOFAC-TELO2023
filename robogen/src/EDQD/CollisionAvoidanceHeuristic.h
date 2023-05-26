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
/**
 * \brief Class for describing a collision avoidance heuristic
 *
 * CollisionAvoidanceHeuristic inherits from Heuristic to describe a heuristic
 * that avoids collisions between robotic agents and other objects in the environments.
 *
 */
class CollisionAvoidanceHeuristic : public Heuristic{
	private:
		/** Flags whether object being avoided is static (true) or moving (false) */
		bool staticObject;
		/** Number of iterations to execute the heuristic for*/
		int avoidCounter;
		/** Flags whether object being avoided is static (true) or moving (false) */
		osg::Vec2d signal;
	public:
		/**
		 * Class constructor
		 * @param Robot The agent to utilize this heuristic
		 * @param Scenario The scenario in which this task is being carried out
		 * @return a heuristic object
		 */
		CollisionAvoidanceHeuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario);
		/**
		 * Destructor
		 */
		~CollisionAvoidanceHeuristic();

		virtual osg::Vec2d step(boost::mutex& queueMutex);
		/**
		 * Steps heuristic when object being avoided is static
		 * @return osg::Vec2d Motor outputs
		 */
		osg::Vec2d stepStatic();
		/**
		 * Update (reduce) number of iterations
		 *
		 */
		void decrement();
		/**
		 * Returns status of detected object - i.e. static or moving
		 * @return bool Flags whether object being avoided is static (true) or moving (false)
		 */
		inline bool isStaticObject(){return staticObject;}
		/**
		 * Sets status of detected object - i.e. static or moving
		 * @param bool Flags whether object being avoided is static (true) or moving (false)
		 */
		inline void setStaticObject(bool value){staticObject = value;}

		/**
		 * Sets number of iterations to keep heuristic active for
		 * @param int number of iterations
		 */
		inline void resetCounter(int value){avoidCounter = value;}
		/**
		 * Returns time this heuristic will be active for
		 * @return int number of iterations left
		 */
		inline int getCounter(){return avoidCounter;}
		/**
		 * Returns motor outputs
		 * @return osg::Vec2d values for left and right motors
		 */
		inline osg::Vec2d getSignal(){return signal;}
};
}



#endif /* EDQD_COLLISIONAVOIDANCEHEURISTIC_H_ */
