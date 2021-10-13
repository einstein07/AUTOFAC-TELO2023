/*
 * @(#) Heuristic.h   1.0   Sep 28, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef EDQD_HEURISTIC_H_
#define EDQD_HEURISTIC_H_

#include <osg/Vec2>
#include <math.h>
#include <boost/shared_ptr.hpp>
//#include "BoxResource.h"
#include "scenario/Scenario.h"
#include "scenario/Environment.h"
#include "Robogen.h"
#include "Robot.h"

namespace robogen{
//class EDQDRobot;
class Heuristic{
	public:
		static constexpr double HALF_PI = M_PI / 2;

		Heuristic(boost::shared_ptr<Robot> robot);
		virtual ~Heuristic();
		void setPriority(int priority);
		virtual osg::Vec2d step(boost::shared_ptr<Environment>& env, boost::shared_ptr<Scenario> scenario)=0;

		osg::Vec2d driveToTargetPosition(osg::Vec2d targetPosition);
		osg::Vec3 getLocalPoint(osg::Vec3 globalPoint);

	protected:
		/**
		 * The agent currently utilizing this heuristic
		 */
		boost::shared_ptr<Robot> robot_;
	private:
		/**
		 * Priority of this heuristic
		 */
		int priority_;

};

}

#endif /* EDQD_HEURISTIC_H_ */
