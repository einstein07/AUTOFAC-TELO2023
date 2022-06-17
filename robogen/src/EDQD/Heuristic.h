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
#include "scenario/Scenario.h"
#include "Util.h"
#include "EDQDRobot.h"
#include "Robogen.h"
#include "Robot.h"

namespace robogen{
class Heuristic{
	public:
		static constexpr double HALF_PI = M_PI / 2;

		Heuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario);
		virtual ~Heuristic();
		void setPriority(int priority);
		virtual osg::Vec2d step()=0;

		osg::Vec2d driveToTargetPosition(osg::Vec2d targetPosition);
		osg::Vec3 getLocalPoint(osg::Vec3 globalPoint);
		osg::Vec3d getGlobalPoint(osg::Vec3d localPoint);

	protected:
		/**
		 * The agent currently utilizing this heuristic
		 */
		boost::shared_ptr<Robot> robot_;
		boost::shared_ptr<Scenario> scenario_;
	private:
		/**
		 * Priority of this heuristic
		 */
		int priority_;

};

}

#endif /* EDQD_HEURISTIC_H_ */
