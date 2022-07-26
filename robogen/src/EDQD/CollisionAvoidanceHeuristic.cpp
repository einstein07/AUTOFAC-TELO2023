/*
 * @(#) CollisionAvoidanceHeuristic.cpp   1.0   Sep 28, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#include "CollisionAvoidanceHeuristic.h"
#include "model/motors/RotationMotor.h"
#include <math.h>
namespace robogen{

	CollisionAvoidanceHeuristic::CollisionAvoidanceHeuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario):
			Heuristic(robot, scenario), staticObject(false), avoidCounter(200){
		setPriority(2);
		signal = osg::Vec2d(-1000, -1000);
	}

	CollisionAvoidanceHeuristic::~CollisionAvoidanceHeuristic(){}

	void CollisionAvoidanceHeuristic::decrement(){
		if (avoidCounter > 0)
			avoidCounter--;
		else{
			//resetCounter();
			setStaticObject(false);
			signal.set(-1000, -1000);
		}
	}

	osg::Vec2d CollisionAvoidanceHeuristic::step(boost::mutex& queueMutex){
		osg::Vec2d pos = osg::Vec2d(-1000, -1000);
		double minDistance = 0.5;//0.35;//0.2;//0.8
		osg::Vec3d area = scenario_ -> getEnvironment() -> getGatheringZone() -> getPosition();

		double d = 0;
		for( unsigned int i = 0 ; i < robot_->getSensors().size(); ++i ){
			if (boost::dynamic_pointer_cast< IrSensorElement>(robot_->getSensors()[i])) {

				// First check if there is any detected object
				if ( boost::dynamic_pointer_cast< IrSensorElement>(robot_->getSensors()[i])->
						read()){
					d = boost::dynamic_pointer_cast< IrSensorElement>(robot_->getSensors()[i])->
							read();
					if (d <= minDistance){
						// If it is another agent, avoid a collision
						if (boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::ROBOT])/** && !robot_ -> isBoundToResource()*/) {
							if ( boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::ROBOT])-> read()){

								int targetIndex = boost::dynamic_pointer_cast<SensorElement>(robot_->getSensors()[i + SensorElement::ROBOT])->getObjectId();
								boost::shared_ptr<Robot> otherRobot = scenario_->getRobot(targetIndex);

								osg::Vec3d otherAgentPos = scenario_->getRobot(targetIndex)->getCoreComponent()->getRootPosition();
								//pos.set(-otherAgentPos.x(), -otherAgentPos.y(), 0);
								/*osg::Vec3d localPos = getLocalPoint(otherAgentPos);
								pos = getGlobalPoint(osg::Vec3d(-10 * localPos.x(), -10 * localPos.y(), -10 * localPos.z()));*/
								double otherRobotMinX, otherRobotMaxX, otherRobotMinY, otherRobotMaxY, otherRobotMinZ, otherRobotMaxZ;
								double robotminX, robotmaxX, robotminY, robotmaxY, robotminZ, robotmaxZ;
								robot_ -> getAABB(robotminX, robotmaxX, robotminY, robotmaxY, robotminZ, robotmaxZ);
								otherRobot -> getAABB( otherRobotMinX,  otherRobotMaxX,  otherRobotMinY,  otherRobotMaxY,  otherRobotMinZ,  otherRobotMaxZ);

								if ((robotminX <  otherRobotMinX && robotmaxX >  otherRobotMaxX) || (robotminX >  otherRobotMinX && robotminX <  otherRobotMaxX)
											|| (robotmaxX >  otherRobotMinX && robotmaxX <  otherRobotMaxX)) {
									setStaticObject(true);
									resetCounter(200);
									osg::Vec3d robotPos = robot_ -> getCoreComponent() -> getRootPosition();
									if (distance(robotPos, osg::Vec3d( otherRobotMinX,  otherRobotMaxY, 0)) < distance(robotPos, osg::Vec3d( otherRobotMaxX,  otherRobotMaxY, 0))){
										pos = osg::Vec2d( otherRobotMinX - 2,  /**otherRobotMaxY*/ robotPos.y());
									}else{
										pos = osg::Vec2d( otherRobotMaxX + 2,  /**otherRobotMaxY*/ robotPos.y());
									}
								}
								else if (((robotminY <  otherRobotMinY && robotmaxY >  otherRobotMaxY) || (robotminY >  otherRobotMinY && robotminY <  otherRobotMaxY)
										|| (robotmaxY >  otherRobotMinY && robotmaxY <  otherRobotMaxY)) && (robotminY >  otherRobotMaxY) ) {

									osg::Vec3d robotPos = robot_ -> getCoreComponent() -> getRootPosition();
									if (distance(robotPos, osg::Vec3d( otherRobotMinY,  otherRobotMaxX, 0)) < distance(robotPos, osg::Vec3d( otherRobotMaxY,  otherRobotMaxX, 0))){
										pos = osg::Vec2d( /**otherRobotMaxX*/ robotPos.x() ,  otherRobotMinY - 2);
									}else{
										pos = osg::Vec2d( /**otherRobotMaxX*/ robotPos.x(),  otherRobotMaxY + 2);
									}
								}
								minDistance = d;
								//pos = osg::Vec3d(-10 * otherAgentPos.x(), -10 * otherAgentPos.y(), 0);
							}
						}

						// If it is a wall, avoid a collision
						if (boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::WALL])) {
							if ( boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::WALL])->
									read()){
								return Heuristic::driveToTargetPosition(osg::Vec2d(0, 0));
							}
						}
					}
					// If it is a large resource and attached to resource, move around
					if (boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET2]) ||
						boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET3]) ||
						boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET4]) ||
						boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET5])    ) {
						int targetIndex = -1;

						if ( boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET2])-> read()){
							if (robot_-> isBoundToResource())
								targetIndex = boost::dynamic_pointer_cast<SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET2]) -> getObjectId() == robot_ -> getBoundResourceId()? -1: boost::dynamic_pointer_cast<SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET2]) -> getObjectId();
							else if (!robot_-> isBoundToResource())
								targetIndex = boost::dynamic_pointer_cast<SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET2]) -> getObjectId();
						}
						if ( boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET3])-> read()){
							if (robot_-> isBoundToResource())
								targetIndex = boost::dynamic_pointer_cast<SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET3]) -> getObjectId() == robot_ -> getBoundResourceId()? -1: boost::dynamic_pointer_cast<SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET3]) -> getObjectId();
							else if (!robot_-> isBoundToResource())
								targetIndex = boost::dynamic_pointer_cast<SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET3]) -> getObjectId();
						}
						if ( boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET4])-> read()){
							if (robot_-> isBoundToResource())
								targetIndex = boost::dynamic_pointer_cast<SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET4]) -> getObjectId() == robot_ -> getBoundResourceId()? -1: boost::dynamic_pointer_cast<SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET4]) -> getObjectId();
							else if (!robot_-> isBoundToResource())
								targetIndex = boost::dynamic_pointer_cast<SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET4]) -> getObjectId();
						}
						if ( boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET5])-> read()){
							if (robot_-> isBoundToResource())
								targetIndex = boost::dynamic_pointer_cast<SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET5]) -> getObjectId() == robot_ -> getBoundResourceId()? -1: boost::dynamic_pointer_cast<SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET5]) -> getObjectId();
							else if (!robot_ ->isBoundToResource())
								targetIndex = boost::dynamic_pointer_cast<SensorElement>(robot_->getSensors()[i + SensorElement::RESOURCET5]) -> getObjectId();
						}
						if (targetIndex != -1){
							boost::shared_ptr<BoxResource> resource = scenario_->getEnvironment() -> getResources()[targetIndex];
							if (robot_ ->isBoundToResource() || (!robot_ -> isBoundToResource() && resource -> pushedByMaxRobots())){
								double minX, maxX, minY, maxY, minZ, maxZ;
								double robotminX, robotmaxX, robotminY, robotmaxY, robotminZ, robotmaxZ;
								resource -> getAABB(minX, maxX, minY, maxY, minZ, maxZ);
								robot_ -> getAABB(robotminX, robotmaxX, robotminY, robotmaxY, robotminZ, robotmaxZ);

								if (robot_ -> isBoundToResource()){
									if (((robotminX < minX && robotmaxX > maxX) || (robotminX > minX && robotminX < maxX)
												|| (robotmaxX > minX && robotmaxX < maxX)) && (robotminY > maxY)) {
										setStaticObject(true);
										resetCounter(250);
										//std::cout << "Robot: "<< robot_ -> getId() <<" aabb: min x: " << robotminX << " max x: " << robotmaxX << " min y: " << robotminY << " max y: " << robotmaxY << " ";
										//std::cout << "Resource type: "<< resource -> getType() <<" aabb: min x: " << minX << " max x: " << maxX << " min y: " << minY << " max y: " << maxY;

										osg::Vec3d robotPos = robot_ -> getCoreComponent() -> getRootPosition();
										if (distance(robotPos, osg::Vec3d(minX, maxY, 0)) < distance(robotPos, osg::Vec3d(maxX, maxY, 0))){
											pos = osg::Vec2d(minX - 2, maxY);
											//std::cout << "Driving to: (" << pos.x() << ", " << pos.y() << ")" << std::endl;
										}else{
											pos = osg::Vec2d(maxX + 2, maxY);
											//std::cout << "Driving to: (" << pos.x() << ", " << pos.y() << ")" << std::endl;
										}
									}
								}
								else{
									if ((robotminX < minX && robotmaxX > maxX) || (robotminX > minX && robotminX < maxX)
											|| (robotmaxX > minX && robotmaxX < maxX)) {

										osg::Vec3d robotPos = robot_ -> getCoreComponent() -> getRootPosition();
										setStaticObject(true);
										resetCounter(250);

										if (distance(robotPos, osg::Vec3d(minX, maxY, 0)) < distance(robotPos, osg::Vec3d(maxX, maxY, 0))){
											pos = osg::Vec2d(minX - 2, maxY);
										}else{
											pos = osg::Vec2d(maxX + 2, maxY);
										}
									}
									else if ((robotminX < minX && robotmaxX > maxX) || (robotminX > minX && robotminX < maxX)
											|| (robotmaxX > minX && robotmaxX < maxX) ) {

										setStaticObject(true);
										resetCounter(250);

										osg::Vec3d robotPos = robot_ -> getCoreComponent() -> getRootPosition();
										if (distance(robotPos, osg::Vec3d(minY, maxX, 0)) < distance(robotPos, osg::Vec3d(maxY, maxX, 0))){
											pos = osg::Vec2d(minY - 2, maxX);
										}else{
											pos = osg::Vec2d(maxY + 2, maxX);
										}
									}
								}
							}
							/**else{
								setStaticObject(true);
																	resetCounter(250);
								pos.set(-resource -> getPosition().x(), -resource -> getPosition().y());
							}*/
						}
					}
					/**if (boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::ROBOT])) {
						if ( boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::ROBOT])->read()){
							int targetIndex = boost::dynamic_pointer_cast<SensorElement>(robot_->getSensors()[i + SensorElement::ROBOT])->getObjectId();
							boost::shared_ptr<Robot> otherRobot = scenario_->getRobot(targetIndex);
							if ( !robot_ -> isBoundToResource() && otherRobot -> isBoundToResource()){
								int resourceId = otherRobot -> getBoundResourceId();
								boost::shared_ptr<BoxResource> resource = scenario_->getEnvironment() -> getResources()[resourceId];

								double otherRobotMinX, otherRobotMaxX, otherRobotMinY, otherRobotMaxY, otherRobotMinZ, otherRobotMaxZ;
								double robotminX, robotmaxX, robotminY, robotmaxY, robotminZ, robotmaxZ;
								robot_ -> getAABB(robotminX, robotmaxX, robotminY, robotmaxY, robotminZ, robotmaxZ);

								if (resource -> getType() == 1){

									otherRobot -> getAABB( otherRobotMinX,  otherRobotMaxX,  otherRobotMinY,  otherRobotMaxY,  otherRobotMinZ,  otherRobotMaxZ);

									if ((robotminX <  otherRobotMinX && robotmaxX >  otherRobotMaxX) || (robotminX >  otherRobotMinX && robotminX <  otherRobotMaxX)
												|| (robotmaxX >  otherRobotMinX && robotmaxX <  otherRobotMaxX)) {

										osg::Vec3d robotPos = robot_ -> getCoreComponent() -> getRootPosition();
										if (distance(robotPos, osg::Vec3d( otherRobotMinX,  otherRobotMaxY, 0)) < distance(robotPos, osg::Vec3d( otherRobotMaxX,  otherRobotMaxY, 0))){
											pos = osg::Vec2d( otherRobotMinX - 1,  otherRobotMaxY);
										}else{
											pos = osg::Vec2d( otherRobotMaxX + 1,  otherRobotMaxY);
										}
									}
									else if (((robotminY <  otherRobotMinY && robotmaxY >  otherRobotMaxY) || (robotminY >  otherRobotMinY && robotminY <  otherRobotMaxY)
											|| (robotmaxY >  otherRobotMinY && robotmaxY <  otherRobotMaxY)) && (robotminY >  otherRobotMaxY) ) {

										osg::Vec3d otherRobotPos = otherRobot -> getCoreComponent() -> getRootPosition();
										pos = osg::Vec2d( -otherRobotPos.x(), otherRobotPos.y());
									}

								}
								else{
									double resourceMinX, resourceMaxX, resourceMinY, resourceMaxY, resourceMinZ, resourceMaxZ;
									resource -> getAABB(resourceMinX, resourceMaxX, resourceMinY, resourceMaxY, resourceMinZ, resourceMaxZ);
									if (((robotminX < resourceMinX && robotmaxX > resourceMaxX) || (robotminX > resourceMinX && robotminX < resourceMaxX)
												|| (robotmaxX > resourceMinX && robotmaxX < resourceMaxX)) && (robotminY > resourceMaxY)) {
										setStaticObject(true);
										resetCounter(200);

										osg::Vec3d robotPos = robot_ -> getCoreComponent() -> getRootPosition();
										if (distance(robotPos, osg::Vec3d(resourceMinX, resourceMaxY, 0)) < distance(robotPos, osg::Vec3d(resourceMaxX, resourceMaxY, 0))){
											pos = osg::Vec2d(resourceMinX - 1, resourceMaxY);
										}else{
											pos = osg::Vec2d(resourceMaxX + 1, resourceMaxY);
										}
									}
								}
							}

						}
					}*/
				}
			}
			else if (boost::dynamic_pointer_cast<TargetAreaDetectorElement>(robot_->getSensors()[i])) {
				if ( boost::dynamic_pointer_cast<TargetAreaDetectorElement>(robot_->getSensors()[i])->read()){
						if ( !robot_->isBoundToResource() ){
							return Heuristic::driveToTargetPosition(osg::Vec2d(-area.x(), -area.y()));
						}
				}
			}
		}
		if (pos.x() == -1000){
			return pos;
		}
		else{
			signal = pos;
			return Heuristic::driveToTargetPosition(pos);
		}
	}

	osg::Vec2d CollisionAvoidanceHeuristic::stepStatic(){
		osg::Vec2d pos = osg::Vec2d(-1000, -1000);
		for( unsigned int i = 0 ; i < robot_->getSensors().size(); ++i ){
			if (boost::dynamic_pointer_cast< IrSensorElement>(robot_->getSensors()[i])) {
				// First check if there is any detected object
				if ( boost::dynamic_pointer_cast< IrSensorElement>(robot_->getSensors()[i])->read()){
					double d = boost::dynamic_pointer_cast< IrSensorElement>(robot_->getSensors()[i])-> read();
					if (boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::ROBOT])) {
						if ( boost::dynamic_pointer_cast< SensorElement>(robot_->getSensors()[i + SensorElement::ROBOT])->read()){
							int targetIndex = boost::dynamic_pointer_cast<SensorElement>(robot_->getSensors()[i + SensorElement::ROBOT])->getObjectId();
							boost::shared_ptr<Robot> otherRobot = scenario_->getRobot(targetIndex);
							if ( !robot_ -> isBoundToResource() && otherRobot -> isBoundToResource()){
								int resourceId = otherRobot -> getBoundResourceId();
								boost::shared_ptr<BoxResource> resource = scenario_->getEnvironment() -> getResources()[resourceId];

								double otherRobotMinX, otherRobotMaxX, otherRobotMinY, otherRobotMaxY, otherRobotMinZ, otherRobotMaxZ;
								double robotminX, robotmaxX, robotminY, robotmaxY, robotminZ, robotmaxZ;
								robot_ -> getAABB(robotminX, robotmaxX, robotminY, robotmaxY, robotminZ, robotmaxZ);

								if (resource -> getType() == 1){

									otherRobot -> getAABB( otherRobotMinX,  otherRobotMaxX,  otherRobotMinY,  otherRobotMaxY,  otherRobotMinZ,  otherRobotMaxZ);

									if ((robotminX <  otherRobotMinX && robotmaxX >  otherRobotMaxX) || (robotminX >  otherRobotMinX && robotminX <  otherRobotMaxX)
												|| (robotmaxX >  otherRobotMinX && robotmaxX <  otherRobotMaxX)) {

										osg::Vec3d robotPos = robot_ -> getCoreComponent() -> getRootPosition();
										if (distance(robotPos, osg::Vec3d( otherRobotMinX,  otherRobotMaxY, 0)) < distance(robotPos, osg::Vec3d( otherRobotMaxX,  otherRobotMaxY, 0))){
											pos = osg::Vec2d( otherRobotMinX - 1,  otherRobotMaxY);
										}else{
											pos = osg::Vec2d( otherRobotMaxX + 1,  otherRobotMaxY);
										}
									}
									else if (((robotminY <  otherRobotMinY && robotmaxY >  otherRobotMaxY) || (robotminY >  otherRobotMinY && robotminY <  otherRobotMaxY)
											|| (robotmaxY >  otherRobotMinY && robotmaxY <  otherRobotMaxY)) && (robotminY >  otherRobotMaxY) ) {

										osg::Vec3d otherRobotPos = otherRobot -> getCoreComponent() -> getRootPosition();
										pos = osg::Vec2d( -otherRobotPos.x(), otherRobotPos.y());
									}

								}
								else{
									double resourceMinX, resourceMaxX, resourceMinY, resourceMaxY, resourceMinZ, resourceMaxZ;
									resource -> getAABB(resourceMinX, resourceMaxX, resourceMinY, resourceMaxY, resourceMinZ, resourceMaxZ);
									if (((robotminX < resourceMinX && robotmaxX > resourceMaxX) || (robotminX > resourceMinX && robotminX < resourceMaxX)
												|| (robotmaxX > resourceMinX && robotmaxX < resourceMaxX)) && (robotminY > resourceMaxY)) {
										setStaticObject(true);
										resetCounter(200);

										osg::Vec3d robotPos = robot_ -> getCoreComponent() -> getRootPosition();
										if (distance(robotPos, osg::Vec3d(resourceMinX, resourceMaxY, 0)) < distance(robotPos, osg::Vec3d(resourceMaxX, resourceMaxY, 0))){
											pos = osg::Vec2d(resourceMinX - 1, resourceMaxY);
										}else{
											pos = osg::Vec2d(resourceMaxX + 1, resourceMaxY);
										}
									}
								}
							}

						}
					}
				}
			}
		}
		return pos;

	}
}


