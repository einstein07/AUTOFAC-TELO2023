/*
 * @(#) DropOffHeuristic.cpp   1.0   Nov 04, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#include "DropOffHeuristic.h"
#include "Heuristic.h"
#include "model/motors/RotationMotor.h"

namespace robogen{

	DropOffHeuristic::DropOffHeuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario):
					Heuristic(robot, scenario), env(scenario->getEnvironment()),
					targetAreaPosition_(scenario->getEnvironment()->getGatheringZone()->getPosition()){

		setPriority(1);
	}

	DropOffHeuristic::~DropOffHeuristic(){}

	osg::Vec2d DropOffHeuristic::step(boost::mutex& queueMutex){
		{
			boost::lock_guard<boost::mutex> lock(queueMutex);
			if (robot_->isBoundToResource()){
				if (boost::dynamic_pointer_cast<EDQDRobot>(robot_)){
					if ( robogen::iterations > 0 && robogen::iterations % EDQD::Parameters::evaluationTime == 0
							/**boost::dynamic_pointer_cast<EDQDRobot>(robot_) -> getTimeResourceBound()> EDQD::Parameters::maxTimeResourceBound*/){

							boost::shared_ptr<BoxResource> resource = env->getResources()[robot_->getBoundResourceId()];
							boost::dynamic_pointer_cast<EDQDRobot>(robot_) -> updateFitness(resource->getPosition(), resource -> getSize(), resource -> getValue());
							boost::dynamic_pointer_cast<EDQDRobot>(robot_) -> incResourceCounter(resource -> getType());
							resource -> dropOff();

						//boost::dynamic_pointer_cast<EDQDRobot>(robot_) -> resetTimeResourceBound();
						//return Heuristic::driveToTargetPosition(osg::Vec2d(-resource -> getPosition().x(), -resource -> getPosition().y()));
						//std::cout << "Drop-off heuristic active for resource type - " << resource -> getType() << std::endl;
					}
					/*else{
						boost::dynamic_pointer_cast<EDQDRobot>(robot_) -> incTimeResourceBound();
					}*/
				}
			}
		}

		return osg::Vec2d(-1000, -1000);
	}
}
