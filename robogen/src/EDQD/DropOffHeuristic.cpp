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
					if ( iterations % EDQD::Parameters::evaluationTime == EDQD::Parameters::evaluationTime-1
							&& abs( targetAreaPosition_.y() - boost::dynamic_pointer_cast<EDQDRobot>(robot_) -> getCoreComponent() -> getRoot() -> getPosition().y()) > scenario_ ->getEnvironment()->getGatheringZone() -> getSize().y()/2 ){

							boost::shared_ptr<BoxResource> resource = env->getResources()[robot_->getBoundResourceId()];
							boost::dynamic_pointer_cast<EDQDRobot>(robot_) -> updateFitness(resource->getPosition(), resource -> getSize(), resource -> getValue());
							boost::dynamic_pointer_cast<EDQDRobot>(robot_) -> incResourceCounter(resource -> getType());
							//std::cout << "DH -inc resource counter for robot id: " << robot_ -> getId() << std::endl;
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
