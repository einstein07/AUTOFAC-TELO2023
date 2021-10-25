/*
 * @(#) PickUpPositioningHeuristic.cpp   1.0   Oct 25, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#include "PickUpHeuristic.h"
#include "Heuristic.h"
#include "model/motors/RotationMotor.h"

namespace robogen{

	PickUpHeuristic::PickUpHeuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario):
					Heuristic(robot, scenario), env(scenario->getEnvironment()),
					targetAreaPosition_(scenario->getEnvironment()->getGatheringZone()->getPosition()){

		setPriority(0);
	}

	PickUpHeuristic::~PickUpHeuristic(){}

	osg::Vec2d PickUpHeuristic::step(){
		return osg::Vec2d(-1000, -1000);
	}
}
