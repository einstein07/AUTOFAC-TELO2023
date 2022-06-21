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


		double minDistanceToObject = 0.2;
		int objectId = -1;
		double type = 0.0;
		int t1 = 0;int t2 = 0;int t3 = 0;int t4 = 0;int t5 = 0;
		int idT1 = -1;int idT2 = -1;int idT3 = -1;int idT4 = -1;int idT5 = -1;

		//return Heuristic::driveToTargetPosition(osg::Vec2d(targetAreaPosition_.x(), targetAreaPosition_.y()));

		/**
		 * If this agent is already attached to a resource,
		 * then navigate to the target area
		 */

		if (robot_->isBoundToResource()){

			/**objectId = robot_->getBoundResourceId();
			boost::shared_ptr<BoxResource> resource = env->getResources()[objectId];

			for (unsigned int  i = 0; i < robot_->getSensors().size(); ++i){
				if (boost::dynamic_pointer_cast<TargetAreaDetectorElement>(robot_->getSensors()[i])) {

					if ( boost::dynamic_pointer_cast<TargetAreaDetectorElement>(robot_->getSensors()[i])->
											read()){
							osg::Vec3d area = osg::Vec3d(-1000, targetAreaPosition_.y(), 0);
							if (robot_->isBoundToResource() && abs((distance(robot_->getCoreComponent()->getRootPosition(), osg::Vec3d(robot_->getCoreComponent()->getRootPosition().x(), targetAreaPosition_.y(), 0))) < 0.5) ){
								std::cout << "Bound to resource: Dropping resource" <<std::endl;
								resource -> setCollected(true);
								std::cout << "Bound to resource: resource set to collected" <<std::endl;

								if (boost::dynamic_pointer_cast<EDQDRobot>(robot_)){
									std::cout << "Bound to resource: incrementing resource counter" <<std::endl;
									boost::dynamic_pointer_cast<EDQDRobot>(robot_)->incResourceCounter( resource->getType() );

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
			}*/

			boost::shared_ptr<BoxResource> resource = env->getResources()[robot_ -> getBoundResourceId()];
			if(resource -> pushedByMaxRobots()){
				//resource -> setMovable();
				std::cout << "number of pushing robots allow resource to be moved: " << resource -> getNumberPushingRobots() << std::endl;
				return Heuristic::driveToTargetPosition(osg::Vec2d(robot_->getCoreComponent()->getRootPosition().x(), targetAreaPosition_.y()));
			}
			else{
				//resource -> setFixed();
				std::cout<< "Waiting for help" << std::endl;
				return osg::Vec2d(0.5, 0.5);
			}
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

					if (distance <= minDistanceToObject){
						if (boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET1])) {
							if ( boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET1])->read()
									&& boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET1])->isActive()){
								idT1 = boost::dynamic_pointer_cast< SensorElement>
									(robot_->getSensors()[i + SensorElement::RESOURCET1])->getObjectId();
								t1 = 1;
							}
						}
						if (boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET2])) {
							if ( boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET2])->read()
									&& boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET2])->isActive()){
								idT2 = boost::dynamic_pointer_cast< SensorElement>
									(robot_->getSensors()[i + SensorElement::RESOURCET2])->getObjectId();
								t2 = 1;
							}
						}
						if (boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET3])) {
							if ( boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET3])->read()
									&& boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET3])->isActive()){
								idT3 = boost::dynamic_pointer_cast< SensorElement>
									(robot_->getSensors()[i + SensorElement::RESOURCET3])->getObjectId();
								t3 = 1;
							}
						}
						if (boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET4])) {
							if ( boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET4])->read()
									&& boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET4])->isActive()){
								idT4 = boost::dynamic_pointer_cast< SensorElement>
									(robot_->getSensors()[i + SensorElement::RESOURCET4])->getObjectId();
								t4 = 1;
							}
						}
						if (boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET5])) {
							if ( boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET5])->read()
									&& boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET5])->isActive()){
								idT5 = boost::dynamic_pointer_cast< SensorElement>
									(robot_->getSensors()[i + SensorElement::RESOURCET5])->getObjectId();
								t5 = 1;
							}
						}

					}
				}
			}

			/**
			 * If the detected resource has the maximum number of required robots attached to it - i.e. size == 0
			 * or the obtained resource id is invalid then keep wandering the environment
			 */
			if ( t1 == t2 == t3 == t4 == t5 == 0 ){
				return osg::Vec2d(-1000, -1000);
			}
			/**
			 * Resource detected and there is space for attachment
			 */
			else{
				std::vector < boost::shared_ptr<BoxResource> > avResources;
				if (idT1 != -1 && idT1 < env->getResources().size()){
					avResources.push_back(env->getResources()[idT1]);
				}
				if (idT2 != -1 && idT2 < env->getResources().size()){
					avResources.push_back(env->getResources()[idT2]);
				}
				if (idT3 != -1 && idT3 < env->getResources().size()){
					avResources.push_back(env->getResources()[idT3]);
				}
				if (idT4 != -1 && idT4 < env->getResources().size()){
					avResources.push_back(env->getResources()[idT4]);
				}
				if (idT5 != -1 && idT5 < env->getResources().size()){
					avResources.push_back(env->getResources()[idT5]);
				}
				if(!avResources.empty()){
					boost::shared_ptr<BoxResource> resource = avResources[0];
					double minDistance = distance(resource -> getPosition(), robot_ -> getBodyPart("Core") -> getRootPosition());
					for (boost::shared_ptr<BoxResource> r : avResources){
						double d = distance(r -> getPosition(), robot_ -> getBodyPart("Core") -> getRootPosition());
						if (minDistance > d){
							minDistance = d;
							resource = r;
						}
					}
					if (resource->pickup(robot_)){
						robot_ -> setBoundResourceId(resource -> getId());
						//return Heuristic::driveToTargetPosition(osg::Vec2d(targetAreaPosition_.x(), targetAreaPosition_.y()));

						if(resource -> pushedByMaxRobots()){
							//resource -> setMovable();
							//std::cout << "number of pushing robots allow resource to be moved: " << resource -> getNumberPushingRobots() << std::endl;
							return Heuristic::driveToTargetPosition(osg::Vec2d(targetAreaPosition_.x(), targetAreaPosition_.y()));
						}
						else{
							//resource -> setFixed();
							return osg::Vec2d(0.5, 0.5);
						}

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
				else{
					return osg::Vec2d(-1000, -1000);
				}
			}
		}
	}
}







