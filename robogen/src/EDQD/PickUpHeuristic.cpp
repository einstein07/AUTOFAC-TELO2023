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

	osg::Vec2d PickUpHeuristic::step(boost::mutex& queueMutex){


		if (heuristicpp_ -> isActive()){
			//std::cout << "Robot " << robot_ -> getId() << " stepping p-u-p" << std::endl;
			return heuristicpp_ -> step(queueMutex);
		}


		double minDistanceToObject = 0.3;//0.2;
		int objectId = -1;
		double type = 0.0;
		int t1 = 0;int t2 = 0;int t3 = 0;int t4 = 0;int t5 = 0;
		int idT1 = -1;int idT2 = -1;int idT3 = -1;int idT4 = -1;int idT5 = -1;

		//return Heuristic::driveToTargetPosition(osg::Vec2d(targetAreaPosition_.x(), targetAreaPosition_.y()));

		/*
		 * If this agent is already attached to a resource,
		 * then navigate to the target area
		 */

		if (robot_->isBoundToResource()){

			boost::shared_ptr<BoxResource> resource = env->getResources()[robot_ -> getBoundResourceId()];
			if(resource -> pushedByMaxRobots()){
				//resource -> setMovable();
				//if (resource -> getType() >1)
					//std::cout << "number of pushing robots allow resource to be moved: " << resource -> getNumberPushingRobots() << std::endl;
				return Heuristic::driveToTargetPosition(osg::Vec2d(robot_->getCoreComponent()->getRootPosition().x(), targetAreaPosition_.y()));
			}
			else{
				//resource -> setFixed();
				//std::cout<< "Waiting for help" << std::endl;
				return osg::Vec2d(0.5, 0.5);
			}
		}
		/*
		 * Not bound to any resource. Keep wandering to find resources that still need to be collected
		 */
		else{

			/*
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

			/*
			 * If the detected resource has the maximum number of required robots attached to it - i.e. size == 0
			 * or the obtained resource id is invalid then keep wandering the environment
			 */
			if ( t1 == t2 == t3 == t4 == t5 == 0 ){
				return osg::Vec2d(-1000, -1000);
			}
			/*
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
					{
						boost::lock_guard<boost::mutex> lock(queueMutex);
						if (resource->pickup(robot_)){

							boost::dynamic_pointer_cast<EDQDRobot>(robot_) -> setPickUpPosition(resource -> getPosition());
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

						else if (ENABLE_PICKUP_POSITIONING /*&& resource -> getType() != 1*/ && !resource -> pushedByMaxRobots()){

							if (resource -> isCollected()){
								heuristicpp_ -> setActive(false);
								return osg::Vec2d(-1000, -1000);
							}
							else{
								if (!heuristicpp_ -> isActive()){
									heuristicpp_ -> setResource(resource);
									heuristicpp_ -> setActive(true);
								}
								return heuristicpp_ -> step(queueMutex);
							}
						}

						else{
							return osg::Vec2d(-1000, -1000);
						}
					}
				}
				else{
					return osg::Vec2d(-1000, -1000);
				}
			}
		}
	}
}







