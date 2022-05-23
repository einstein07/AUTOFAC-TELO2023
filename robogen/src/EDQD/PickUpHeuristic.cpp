/*
 * @(#) PickUpHeuristic.cpp   1.0   Sep 28, 2021
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
		heuristicpp_ = boost::shared_ptr<PickUpPositioningHeuristic>(new PickUpPositioningHeuristic(robot, scenario));
		setPriority(1);
	}

	PickUpHeuristic::~PickUpHeuristic(){}

	osg::Vec2d PickUpHeuristic::step(){


		if (heuristicpp_ -> ACTIVE){
			std::cout << "ACTIVE" << std::endl;
			return heuristicpp_ -> step();
		}


		double minDistanceToObject = 0.8;
		int objectId = -1;
		double size = 0.0;

		/**
		 * If this agent is already attached to a resource,
		 * then navigate to the target area
		 */

		if (robot_->isBoundToResource()){

			objectId = robot_->getBoundResourceId();
			boost::shared_ptr<BoxResource> resource = env->getResources()[objectId];

			for (unsigned int  i = 0; i < robot_->getSensors().size(); ++i){
				if (boost::dynamic_pointer_cast<TargetAreaDetectorElement>(robot_->getSensors()[i])) {

					if ( boost::dynamic_pointer_cast<TargetAreaDetectorElement>(robot_->getSensors()[i])->
											read()){
							std::cout << "Bound to resource: Inside target area" <<std::endl;
							osg::Vec3d area = osg::Vec3d(targetAreaPosition_.x(), targetAreaPosition_.y(), 0);
							if (robot_->isBoundToResource() && abs((distance(robot_->getCoreComponent()->getRootPosition(), area)) < 0.5) ){
								std::cout << "Bound to resource: Dropping resource" <<std::endl;
								resource -> setCollected(true);
								std::cout << "Bound to resource: resource set to collected" <<std::endl;

								if (boost::dynamic_pointer_cast<EDQDRobot>(robot_)){
									std::cout << "Bound to resource: incrementing resource counter" <<std::endl;
									boost::dynamic_pointer_cast<EDQDRobot>(robot_)->incResourceCounter( resource->getSize() );

									if (env->getGatheringZone()->addResource( resource )){
										std::cout 	<< "Resource added to gathering zone. Time bound to resource: "
													<< boost::dynamic_pointer_cast<EDQDRobot>(robot_) -> getTimeResourceBound()
													<< std::endl;
										boost::dynamic_pointer_cast<EDQDRobot>(robot_) -> resetTimeResourceBound();
										// Resource dropped, now wonder around looking for a new resource
										return osg::Vec2d(-1000, -1000);

									}

								}
							}


					}
				}
			}
			return Heuristic::driveToTargetPosition(osg::Vec2d(/**targetPos.x()*/targetAreaPosition_.x(), targetAreaPosition_.y()/**targetPos.y()*/));
			/**
			 * Code commented out below only relevant when robots are cooperating
			 */
			/**if (resource -> getNumberPushingRobots() != resource -> getSize()){
				/**std::cout 	<< "Robot - " << robot_ -> getId()
							<< " waiting for help."
							<< std::endl;*/
				//return osg::Vec2d(0, 0);
			/**}
			else{*/
			// Target area has not been detected yet, continue driving to target area
			//}
		}
		/***
		 * Not bound to any resource. Keep wandering to find resources that still need to be collected
		 */
		else{

			/**
			 * Check if there is any object detected by the IR sensors.
			 * If there is, then check its distance to the robot, and then finally check
			 * if the said object is a resource. If the said object is a resource then
			 * obtain its object id and size
			 */

			for (unsigned int  i = 0; i < robot_->getSensors().size(); ++i){
				double distance = 0;

				if (boost::dynamic_pointer_cast< IrSensorElement>(robot_->getSensors()[i])) {

					distance = boost::dynamic_pointer_cast< IrSensorElement>(robot_->getSensors()[i])->
							read();

					if (distance < minDistanceToObject){
						if (boost::dynamic_pointer_cast< ColorSensorElement>(robot_->getSensors()[i+2])) {
							if ( boost::dynamic_pointer_cast< ColorSensorElement>(robot_->getSensors()[i+2])->
									read()){
								objectId = boost::dynamic_pointer_cast< ColorSensorElement>
									(robot_->getSensors()[i+3])->getObjectId();
								size = boost::dynamic_pointer_cast< ColorSensorElement>(robot_->getSensors()[i+3])->
										read();
							}
						}

					}
				}
			}

			/**
			 * If the detected resource has the maximum number of required robots attached to it - i.e. size == 0
			 * or the obtained resource id is invalid then keep wandering the environment
			 */
			if ( objectId < 0 || size == 0.0 || objectId > env->getResources().size() ){
				return osg::Vec2d(-1000, -1000);
			}
			/**
			 * Resource detected and there is space for attachment
			 */
			else{
				boost::shared_ptr<BoxResource> resource = env->getResources()[objectId];

				if (resource->attachRobot(robot_)){
					robot_ -> setBoundResourceId(objectId);
					return Heuristic::driveToTargetPosition(osg::Vec2d(targetAreaPosition_.x(), targetAreaPosition_.y()));
				}
					/**
					 * This code is only relevant for when robots are cooperating
					 */
					/**if (resource -> getNumberPushingRobots() != resource -> getSize()){
						return osg::Vec2d(0, 0);
					}
					else{*/
						// Successfully attached to resource, now drive to target area to drop off resource
					//}


				else if (ENABLE_PICKUP_POSITIONING ){
					std::cout << "resource could not be collected ";
					if (resource -> isCollected()){
						std::cout << "because it has been collected." <<std::endl;
						return osg::Vec2d(-1000, -1000);

					}
					else{
						heuristicpp_ -> setResource(resource);

						return heuristicpp_ -> step();
					}
				}
				else{
					return osg::Vec2d(-1000, -1000);
				}
			}
		}
	}
}







