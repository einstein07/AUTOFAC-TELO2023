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
/**
 * \brief Class for describing task specific heuristics
 *
 * Heuristic describes the main entities required to create a heuristic of any kind and should be used by
 * classes that describe a heuristic of any kind.
 */
class Heuristic{
	public:
		/** \brief Value of half PI */
		static constexpr double HALF_PI = M_PI / 2;

		/**
		 * \brief Class constructor
		 * @param Robot The agent to utilize this heuristic
		 * @param Scenario The scenario in which this task is being carried out
		 * @return a heuristic object
		 */
		Heuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario);
		/*
		 * \brief Destructor
		 */
		virtual ~Heuristic();
		/**
		 * \brief Sets the priority of this heuristic
		 */
		void setPriority(int priority);
		/**
		 * \brief Update the heuristic at this iteration
		 * @param boost::mutex a lock to avoid race conditions
		 */
		virtual osg::Vec2d step(boost::mutex& queueMutex)=0;

		/**
		 * \brief Returns motor signals that will drive the agent towards the desired
		 * target position
		 * @param osg::Vec2d The position of the target in local coordinates
		 * @return osg::Vec2d The heuristic motor signals
		 */
		osg::Vec2d driveToTargetPosition(osg::Vec2d targetPosition);
		/**
		 * \brief Converts global coordinates to local coordinates, relative to the robotic agent
		 * @param osg::Vec3 The global point
		 * @return osg::Vec3 The local point
		 */
		osg::Vec3 getLocalPoint(osg::Vec3 globalPoint);
		/**
		 * \brief Converts local coordinates (relative to the robotic agent) to global coordinates
		 * @param osg::Vec3 The local point
		 * @return osg::Vec3 The global point
		 */
		osg::Vec3d getGlobalPoint(osg::Vec3d localPoint);

	protected:
		/** \brief The agent currently utilizing this heuristic */
		boost::shared_ptr<Robot> robot_;
		/** \brief The scenario in which the task is being carried out */
		boost::shared_ptr<Scenario> scenario_;
	private:
		/**
		 * Priority of this heuristic
		 */
		int priority_;

};

}

#endif /* EDQD_HEURISTIC_H_ */
