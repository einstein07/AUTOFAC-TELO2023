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

	PickUpHeuristic::PickUpHeuristic(boost::shared_ptr<Robot> robot, osg::Vec2d targetAreaPosition):Heuristic(robot){
		targetAreaPosition_ = targetAreaPosition;
		setPriority(1);
	}

	PickUpHeuristic::~PickUpHeuristic(){}

	osg::Vec2d PickUpHeuristic::step(boost::shared_ptr<Environment>& env, boost::shared_ptr<Scenario> scenario){
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
		double minDistanceToObject = 10000000;
		ColorSensorElement::Type colorType;
		IrSensorElement::Type IRtype;
		int objectId = -1;
		double size = 0.0;
		if (robot_->isBoundToResource()){
			for (unsigned int  i = 0; i < robot_->getSensors().size(); ++i){
				if (boost::dynamic_pointer_cast< ColorSensorElement>(robot_->getSensors()[i])) {
					colorType = boost::dynamic_pointer_cast< ColorSensorElement>(robot_->getSensors()[i])->
																getType();
					/**std::cout << "It is a color sensor element. Value: "
							<< boost::dynamic_pointer_cast< ColorSensorElement>(robot_->getSensors()[i])->
							read()
							<< ". Type: "
							<< colorType
							<< std::endl;*/
					if ( boost::dynamic_pointer_cast< ColorSensorElement>(robot_->getSensors()[i])->
											read()){
						colorType = boost::dynamic_pointer_cast< ColorSensorElement>(robot_->getSensors()[i])->
												getType();
						//std::cout << "Sensor number: "<< i << "is of type: " << colorType << std::endl;
						if (colorType == ColorSensorElement::TARGETAREA){
							std::cout << "Target area detected." << std::endl;
							if (robot_->isBoundToResource()){
								boost::shared_ptr<BoxResource> resource = env->getResources()[robot_->getBoundResourceId()];
								resource->setCollected(true);
							}
							return osg::Vec2d(-1000, -1000);
						}
					}
				}
			}
			return Heuristic::driveToTargetPosition(osg::Vec2d(/**targetPos.x()*/targetAreaPosition_.x(), targetAreaPosition_.y()/**targetPos.y()*/));
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

		if (objectId == -1 || size == 0.0){
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
				//std::cout << "Object picked up" << std::endl;
				// Successfully attached to resource, now drive to target area to drop off resource
				return Heuristic::driveToTargetPosition(osg::Vec2d(/**targetPos.x()*/targetAreaPosition_.x(), targetAreaPosition_.y()/**targetPos.y()*/));
			}
			else if (ENABLE_PICKUP_POSITIONING){
				// TODO: Add code logic
			}
		}

	}

}







