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

		/**
		osg::Vec3d slotAxis = robot_ -> getCoreComponent() -> getSlotAxis(2);
		osg::Vec3d slotPosition = robot_ -> getCoreComponent() -> getSlotPosition(2);
		osg::Vec3d slotOrientation = robot_ -> getCoreComponent() -> getSlotOrientation(2);

		std::cout 	<< "Slot axis: ("
					<< slotAxis.x() << ", "
					<< slotAxis.y() << ", "
					<< slotAxis.z() << ").\nSlot position: ("
					<< slotPosition.x() << ", "
					<< slotPosition.y() << ", "
					<< slotPosition.z() << ").\nSlot orientation: ("
					<< slotOrientation.x() << ", "
					<< slotOrientation.y() << ", "
					<< slotOrientation.z() << ")"
					<< std::endl;*/

		if (heuristicpp_ -> ACTIVE){
			std::cout << "ACTIVE" << std::endl;
			return heuristicpp_ -> step();
		}

		/**
		 * If this agent is already attached to a resource,
		 * then navigate to the target area
		 */
		osg::Vec3d targetPos =
						Heuristic::getLocalPoint(
												osg::Vec3f(
															targetAreaPosition_.x(),
															targetAreaPosition_.y(),
															0
														)
												);
		double minDistanceToObject = 1/*0.05*/;
		ColorSensorElement::Type colorType;
		IrSensorElement::Type IRtype;
		int objectId = -1;
		double size = 0.0;
		if (robot_->isBoundToResource()){
			/**std::cout 	<< "Bound to resource. Getting resouce from environment. Resource id: "
						<< robot_->getBoundResourceId()
						<< std::endl;*/
			boost::shared_ptr<BoxResource> resource = env->getResources()[robot_->getBoundResourceId()];
			//std::cout << "Done" <<std::endl;
			for (unsigned int  i = 0; i < robot_->getSensors().size(); ++i){
				if (boost::dynamic_pointer_cast<TargetAreaDetectorElement>(robot_->getSensors()[i])) {
					/**colorType = boost::dynamic_pointer_cast< ColorSensorElement>(robot_->getSensors()[i])->
																getType();*/
					/**std::cout << "It is a target area detector element. Value: "
							<< boost::dynamic_pointer_cast<TargetAreaDetectorElement>(robot_->getSensors()[i])->
							read()
							<< std::endl;*/
					if ( boost::dynamic_pointer_cast<TargetAreaDetectorElement>(robot_->getSensors()[i])->
											read()){
						/**colorType = boost::dynamic_pointer_cast< ColorSensorElement>(robot_->getSensors()[i])->
												getType();*/
						//std::cout << "Sensor number: "<< i << "is of type: " << colorType << std::endl;
						//if (colorType == ColorSensorElement::TARGETAREA){
							//std::cout << "Target area detected." << std::endl;
							osg::Vec3d area = osg::Vec3d(targetAreaPosition_.x(), targetAreaPosition_.y(), 0);
							if (robot_->isBoundToResource() && abs((distance(robot_->getCoreComponent()->getRootPosition(), area)) < 0.5) ){

								resource->setCollected(true);
								/**std::cout 	<< "resource set to collected."
											<< std::endl;*/
								if (boost::dynamic_pointer_cast<EDQDRobot>(robot_)){
									boost::dynamic_pointer_cast<EDQDRobot>(robot_)->incResourceCounter(resource->getSize());
									/**std::cout 	<< "resource counter incremented."
												<< std::endl;*/
									env->getGatheringZone()->addResource(resource);
									if (boost::dynamic_pointer_cast<EDQDRobot>(robot_)){

										std::cout 	<< "Resource added to gathering zone. Time bound to resource: "
													<< boost::dynamic_pointer_cast<EDQDRobot>(robot_) -> getTimeResourceBound()
													<< std::endl;
										boost::dynamic_pointer_cast<EDQDRobot>(robot_) -> resetTimeResourceBound();
									}

								}
							}
							return osg::Vec2d(-1000, -1000);
						//}
					}
				}
			}
			if (resource -> getNumberPushingRobots() != resource -> getSize()){
				/**std::cout 	<< "Robot - " << robot_ -> getId()
							<< " waiting for help."
							<< std::endl;*/
				return osg::Vec2d(0, 0);
			}
			else{
				return Heuristic::driveToTargetPosition(osg::Vec2d(/**targetPos.x()*/targetAreaPosition_.x(), targetAreaPosition_.y()/**targetPos.y()*/));
			}
		}

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
		//std::cout << "Drive to target area." << std::endl;
		//return Heuristic::driveToTargetPosition(osg::Vec2d(/**targetPos.x()*/targetAreaPosition_.x(), targetAreaPosition_.y()/**targetPos.y()*/));

		if (objectId == -1 || size == 0.0 ){
			return osg::Vec2d(-1000, -1000);
		}
		else{ //Object detected and there is space for attachment
		//	std::cout << "Object detected" << std::endl;
			boost::shared_ptr<BoxResource> resource = env->getResources()[objectId];


			// **************************

			// Send control to motors
			/*for (unsigned int i = 0; i < robot_->getMotors().size(); ++i) {

				if (boost::dynamic_pointer_cast<RotationMotor>(robot_->getMotors()[i])) {
					boost::dynamic_pointer_cast<RotationMotor>(robot_->getMotors()[i])
							->setDesiredVelocity(
													0.0,
													0.1
												);
				}

			}
			for (unsigned int i = 0; i < robot_ -> getMotors().size(); ++i) {
				robot_ -> getMotors()[i]->step(0.1 ) ; //* configuration->getActuationPeriod() );
			}*/
			// ***************************


			if (resource->pickup(robot_)){
				robot_ -> setBoundResourceId(objectId);
				/**std::cout 	<< "Robot - "
							<< robot_ -> getId()
							<< " picked up resource" << std::endl;*/
				if (resource -> getNumberPushingRobots() != resource -> getSize()){
					return osg::Vec2d(0, 0);
				}
				else{
					// Successfully attached to resource, now drive to target area to drop off resource
					return Heuristic::driveToTargetPosition(osg::Vec2d(/**targetPos.x()*/targetAreaPosition_.x(), targetAreaPosition_.y()/**targetPos.y()*/));
				}
			}

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







