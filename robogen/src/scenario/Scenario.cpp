/*
 * @(#) Scenario.cpp   1.0   Mar 13, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#include <iostream>
#include <vector>
#include "config/RobogenConfig.h"
#include "config/TerrainConfig.h"
#include "model/objects/BoxObstacle.h"
#include "scenario/Scenario.h"
#include "scenario/Terrain.h"
#include "Robot.h"
#include "Environment.h"
#include "model/objects/BoxResource.h"
#include "utils/RobogenUtils.h"

namespace robogen {

Scenario::Scenario(boost::shared_ptr<RobogenConfig> robogenConfig) :
		robogenConfig_(robogenConfig), startPositionId_(0),
		stopSimulationNow_(false) {

}

Scenario::~Scenario() {

}

bool Scenario::init(dWorldID odeWorld, dSpaceID odeSpace,
		boost::shared_ptr<Robot> robot) {
        
        environment_ = boost::shared_ptr<Environment>(new
                    Environment(odeWorld, odeSpace, robogenConfig_));

        if(!environment_->init()) {
                return false;
        }

        stopSimulationNow_ = false;
  
	
	robot_ = robot;

	// Setup robot position
	double minX = 0;
	double maxX = 0;
	double minY = 0;
	double maxY = 0;
	double minZ = 0;
	double maxZ = 0;

	// Starting position and orientation
	osg::Vec3 startingPosition =
			robogenConfig_->getStartingPos()->getStartPosition(
					startPositionId_)->getPosition();
	float startingAzimuth = robogenConfig_->getStartingPos()->getStartPosition(
			startPositionId_)->getAzimuth();
	osg::Quat roboRot;
	roboRot.makeRotate(osg::inDegrees(startingAzimuth), osg::Vec3(0,0,1));

	robot->rotateRobot(roboRot);
	robot->getAABB(minX, maxX, minY, maxY, minZ, maxZ);
	robot->translateRobot(
			osg::Vec3(startingPosition.x(),
					startingPosition.y(),
					robogenConfig_->getTerrainConfig()->getHeight()
						+ inMm(2) - minZ));
	robot->getAABB(minX, maxX, minY, maxY, minZ, maxZ);

	std::cout
			<< "The robot is enclosed in the AABB(minX, maxX, minY, maxY, minZ, maxZ) ("
			<< minX << ", " << maxX << ", " << minY << ", " << maxY << ", "
			<< minZ << ", " << maxZ << ")" << std::endl;
	std::cout << "Obstacles in this range will not be generated" << std::endl << std::endl;

	// Setup obstacles
	boost::shared_ptr<ObstaclesConfig> obstacles =
			robogenConfig_->getObstaclesConfig();

	// Instance the boxes above the maximum terrain height
	const std::vector<osg::Vec3>& obstacleCoordinates = obstacles->getCoordinates();
	const std::vector<osg::Vec3>& obstacleSizes = obstacles->getSizes();
	const std::vector<float>& d = obstacles->getDensities();
	const std::vector<osg::Vec3>& rotationAxis = obstacles->getRotationAxes();
	const std::vector<float>& rotationAngles = obstacles->getRotationAngles();

	obstaclesRemoved_ = false;

	double overlapMaxZ=minZ;

	for (unsigned int i = 0; i < obstacleCoordinates.size(); ++i) {
		boost::shared_ptr<BoxObstacle> obstacle(
									new BoxObstacle(odeWorld, odeSpace,
											obstacleCoordinates[i],
											obstacleSizes[i], d[i], rotationAxis[i],
											rotationAngles[i]));
		double oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ;
		obstacle->getAABB(oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ);

		// Do not insert the obstacle if it is in the robot range
		bool inRangeX = false;
		if ((oMinX <= minX && oMaxX >= maxX) || (oMinX >= minX && oMinX <= maxX)
				|| (oMaxX >= minX && oMaxX <= maxX)) {
			inRangeX = true;
		}

		bool inRangeY = false;
		if ((oMinY <= minY && oMaxY >= maxY) || (oMinY >= minY && oMinY <= maxY)
				|| (oMaxY >= minY && oMaxY <= maxY)) {
			inRangeY = true;
		}

		bool inRangeZ = false;
		if ((oMinZ <= minZ && oMaxZ >= maxZ) || (oMinZ >= minZ && oMinZ <= maxZ)
				|| (oMaxZ >= minZ && oMaxZ <= maxZ)) {
			inRangeZ = true;
		}

		// Do not insert obstacles in the robot range
		if (!(inRangeX && inRangeY && inRangeZ)) {
			environment_->addObstacle(obstacle);
		} else {
			if (robogenConfig_->getObstacleOverlapPolicy() ==
					RobogenConfig::ELEVATE_ROBOT) {

				if (oMaxZ > overlapMaxZ)
					overlapMaxZ = oMaxZ;
				environment_->addObstacle(obstacle);

			} else {
				obstacle->remove();
				obstaclesRemoved_ = true;
			}
		}

	}

	// Setup light sources
	boost::shared_ptr<LightSourcesConfig> lightSourcesConfig =
			robogenConfig_->getLightSourcesConfig();

	std::vector<boost::shared_ptr<LightSource> > lightSources;

	std::vector<osg::Vec3> lightSourcesCoordinates =
			lightSourcesConfig->getCoordinates();
	std::vector<float> lightSourcesIntensities =
				lightSourcesConfig->getIntensities();

	for (unsigned int i = 0; i < lightSourcesCoordinates.size(); ++i) {
		double lMinX = lightSourcesCoordinates[i].x() - LightSource::RADIUS;
		double lMaxX = lightSourcesCoordinates[i].x() + LightSource::RADIUS;
		double lMinY = lightSourcesCoordinates[i].y() - LightSource::RADIUS;
		double lMaxY = lightSourcesCoordinates[i].y() + LightSource::RADIUS;
		double lMinZ = lightSourcesCoordinates[i].z() - LightSource::RADIUS;
		double lMaxZ = lightSourcesCoordinates[i].z() + LightSource::RADIUS;

		// Do not insert the ligh source if it is in the robot range
		bool inRangeX = false;
		if ((lMinX <= minX && lMaxX >= maxX) || (lMinX >= minX && lMinX <= maxX)
				|| (lMaxX >= minX && lMaxX <= maxX)) {
			inRangeX = true;
		}

		bool inRangeY = false;
		if ((lMinY <= minY && lMaxY >= maxY) || (lMinY >= minY && lMinY <= maxY)
				|| (lMaxY >= minY && lMaxY <= maxY)) {
			inRangeY = true;
		}

		bool inRangeZ = false;
		if ((lMinZ <= minZ && lMaxZ >= maxZ) || (lMinZ >= minZ && lMinZ <= maxZ)
				|| (lMaxZ >= minZ && lMaxZ <= maxZ)) {
			inRangeZ = true;
		}


		if (!(inRangeX && inRangeY && inRangeZ)) {
			lightSources.push_back(boost::shared_ptr<LightSource>(
						new LightSource(odeSpace, lightSourcesCoordinates[i],
								lightSourcesIntensities[i])));
		} else {
			if (robogenConfig_->getObstacleOverlapPolicy() ==
					RobogenConfig::ELEVATE_ROBOT) {

				if (lMaxZ > overlapMaxZ)
					overlapMaxZ = lMaxZ;
				lightSources.push_back(boost::shared_ptr<LightSource>(
						new LightSource(odeSpace, lightSourcesCoordinates[i],
								lightSourcesIntensities[i])));
			} else {
				obstaclesRemoved_ = true;
			}
		}



	}
	environment_->setLightSources(lightSources);



	if (robogenConfig_->getObstacleOverlapPolicy() ==
			RobogenConfig::ELEVATE_ROBOT) {

		robot->translateRobot(
				osg::Vec3(startingPosition.x(), startingPosition.y(),
						overlapMaxZ + inMm(2) - minZ));
	}




	// optimize the physics!  replace all fixed joints with composite bodies
	robot->optimizePhysics();

	return true;
}

/**
* SM added: Initializes a scenario with multiple robots
*
* @param odeWorld
* @param odeSpace
* @param robot-swarm
* @return true if the scenario was initialized successfully, false otherwise
*/
bool Scenario::init(dWorldID odeWorld, dSpaceID odeSpace, dSpaceID areaSpace,
               std::vector<boost::shared_ptr<Robot> > robots){
        environment_ = boost::shared_ptr<Environment>(new
			Environment(odeWorld, odeSpace, areaSpace, robogenConfig_));
	if(!environment_->init()) {
		return false;
	}
	terrain_ = environment_ -> getTerrainID();
	stopSimulationNow_ = false;
	boost::shared_ptr<GatheringZone> gatheringZone = boost::shared_ptr<GatheringZone>(
				new GatheringZone(
						odeWorld,
						areaSpace,
						robogenConfig_->getGatheringZoneConfig() -> getPosition(),
						robogenConfig_->getGatheringZoneConfig() -> getSize(),
						robogenConfig_->getGatheringZoneConfig() -> getRotationAxis(),
						robogenConfig_->getGatheringZoneConfig() -> getRotationAngle()
						));
	environment_ -> setGatheringZone(gatheringZone);
        /**---------------------------------------------------------------------
         * Setup robot positions - inner vector contains: 
         * minX;maxX;minY;maxY;minZ;maxZ; in that order
         * ---------------------------------------------------------------------
         */
        std::vector<std::vector<double> > robots_position;
        std::vector<osg::Vec3 > starting_positions;
        std::vector<float > starting_azimuths;
        for (int i = 0; i < robots.size(); ++i){
            std::vector<double> robot_position; //inner vector - temporary
            robot_position.push_back(0);//minX
            robot_position.push_back(0);//maxX
            robot_position.push_back(0);//minY
            robot_position.push_back(0);//maxY
            robot_position.push_back(0);//minZ
            robot_position.push_back(0);//maxZ
            robots_position.push_back(robot_position);
            
            robots_.push_back(robots[i]); 
            
            // Starting position and orientation
            starting_positions.push_back(
                            robogenConfig_->getStartingPos()->getStartPosition(
                                            i)->getPosition()
                                        );
            starting_azimuths.push_back(
                            robogenConfig_->getStartingPos()->getStartPosition(
                                            i)->getAzimuth()
                                        );
            osg::Quat roboRot;
            roboRot.makeRotate(osg::inDegrees(starting_azimuths[i]), osg::Vec3(0,0,1));

            robots[i]->rotateRobot(roboRot);
            robots[i]->getAABB(
                robots_position[i][0], //minX
                robots_position[i][1], //maxX 
                robots_position[i][2], //minY
                robots_position[i][3], //maxY
                robots_position[i][4], //minZ
                robots_position[i][5]  //maxZ
            );
            robots[i]->translateRobot(
                            osg::Vec3(starting_positions[i].x(),
                                            starting_positions[i].y(),
                                            robogenConfig_->getTerrainConfig()->getHeight() + starting_positions[i].z() //0.69
                                                    + inMm(2) - robots_position[i][4]));
            robots[i]->getAABB(
                robots_position[i][0], //minX
                robots_position[i][1], //maxX 
                robots_position[i][2], //minY
                robots_position[i][3], //maxY
                robots_position[i][4], //minZ
                robots_position[i][5]  //maxZ
            );

        }
        /**---------------------------------------------------------------------
         * Setup obstacles
         * ---------------------------------------------------------------------  
         */
        boost::shared_ptr<ObstaclesConfig> obstacles =
			robogenConfig_->getObstaclesConfig();

	// Instance the boxes above the maximum terrain height
    int obstacleId = 0;
	const std::vector<osg::Vec3>& obstacleCoordinates = obstacles->getCoordinates();
	const std::vector<osg::Vec3>& obstacleSizes = obstacles->getSizes();
	const std::vector<float>& obstacleDesnties = obstacles->getDensities();
	const std::vector<osg::Vec3>& obstacleRotationAxis = obstacles->getRotationAxes();
	const std::vector<float>& obstacleRotationAngles = obstacles->getRotationAngles();

	obstaclesRemoved_ = false;

        double overlapMaxZ = 0;//minZ;
	for (unsigned int i = 0; i < obstacleCoordinates.size(); ++i) {
		boost::shared_ptr<BoxObstacle> obstacle(
									new BoxObstacle(odeWorld, odeSpace,	obstacleCoordinates[i],
											obstacleSizes[i], obstacleDesnties[i], obstacleRotationAxis[i],
											obstacleRotationAngles[i], obstacleId));
		double oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ;
		obstacle->getAABB(oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ);
                // Do not insert the obstacle if it is in the robot range
                bool inRangeX = false;
                bool inRangeY = false;
                bool inRangeZ = false;
                for (unsigned int  j = 0; j < robots.size(); ++j){
                    overlapMaxZ = (robots_position[j][4] > overlapMaxZ) ? robots_position[j][4] : overlapMaxZ;//minZ
                    if ((oMinX <= robots_position[j][0] && oMaxX >= robots_position[j][1]) 
                            || (oMinX >= robots_position[j][0] && oMinX <= robots_position[j][1])
                                    || (oMaxX >= robots_position[j][0] && oMaxX <= robots_position[j][1])) {
                            inRangeX = true;
                    }

                    if ((oMinY <= robots_position[j][2] && oMaxY >= robots_position[j][3]) 
                            || (oMinY >= robots_position[j][2] && oMinY <= robots_position[j][3])
                                    || (oMaxY >= robots_position[j][2] && oMaxY <= robots_position[j][3])) {
                            inRangeY = true;
                    }

                    if ((oMinZ <= robots_position[j][4] && oMaxZ >= robots_position[j][5]) 
                            || (oMinZ >= robots_position[j][4] && oMinZ <= robots_position[j][5])
                                    || (oMaxZ >= robots_position[j][4] && oMaxZ <= robots_position[j][5])) {
                            inRangeZ = true;
                    }

                }

		// Do not insert obstacles in the robot range
		if (!(inRangeX && inRangeY && inRangeZ) || obstacleRotationAngles[i] >= RobogenUtils::EPSILON_2) {
			environment_->addObstacle(obstacle);
		} else {
			if (robogenConfig_->getObstacleOverlapPolicy() ==
					RobogenConfig::ELEVATE_ROBOT) {

				if (oMaxZ > overlapMaxZ)
					overlapMaxZ = oMaxZ;
				environment_->addObstacle(obstacle);

			} else {
				obstacle->remove();
				obstaclesRemoved_ = true;
			}
		}

	}//end of obstacles loop
        
	/**---------------------------------------------------------------------
	 * Setup resources
	 *----------------------------------------------------------------------
	 */
	boost::shared_ptr<ResourcesConfig> resources =
		robogenConfig_->getResourcesConfig();

	// Instance the boxes above the maximum terrain height
	int resourceId = 0;
	const std::vector<osg::Vec3>& resourceCoordinates = resources->getCoordinates();
	const std::vector<osg::Vec3>& resourceSizes = resources->getSizes();
	const std::vector<float>& resourceDensities = resources->getDensities();
	const std::vector<osg::Vec3>& resourceRotationAxis = resources->getRotationAxes();
	const std::vector<float>& resourceRotationAngles = resources->getRotationAngles();
	const std::vector<int>& types = resources->getTypes();

	resourcesRemoved_ = false;

	overlapMaxZ = 0;//minZ;
	std::cout << "Task environment width: " << environment_ -> getTerrain() -> getWidth() << std::endl;// << " Task environment length: " << environment_ -> getTerrai
	std::cout << "Total number of resources from config file: " << resourceCoordinates.size() << std::endl;
	for (unsigned int i = 0; i < resourceCoordinates.size(); ++i) {
		boost::shared_ptr<BoxResource> resource(
                                                new BoxResource(
                                                            odeWorld, 
                                                            //perResourceSpace[i],
                                                            odeSpace,
															resourceCoordinates[i],
                                                            resourceSizes[i], 
															resourceDensities[i],
															resourceRotationAxis[i],
															resourceRotationAngles[i],
                                                            types[i],
															resourceId
                                                            )
                                                        );
		double rMinX, rMaxX, rMinY, rMaxY, rMinZ, rMaxZ;
		resource->getAABB(rMinX, rMaxX, rMinY, rMaxY, rMinZ, rMaxZ);
		// Do not insert the resource if it is in the robot range
		bool inRangeX = false;
		bool inRangeY = false;
		bool inRangeZ = false;
		for (unsigned int  j = 0; j < robots.size(); ++j){
			overlapMaxZ = (robots_position[j][4] > overlapMaxZ) ?
								robots_position[j][4] : overlapMaxZ;//minZ
			if ((rMinX <= robots_position[j][0] &&
										rMaxX >= robots_position[j][1])
					|| (rMinX >= robots_position[j][0] &&
										rMinX <= robots_position[j][1])
					|| (rMaxX >= robots_position[j][0] &&
										rMaxX <= robots_position[j][1])) {
					inRangeX = true;
			}

			if ((rMinY <= robots_position[j][2] &&
										rMaxY >= robots_position[j][3])
					|| (rMinY >= robots_position[j][2] &&
										rMinY <= robots_position[j][3])
					|| (rMaxY >= robots_position[j][2] &&
										rMaxY <= robots_position[j][3])) {
					inRangeY = true;
			}

			if ((rMinZ <= robots_position[j][4] &&
										rMaxZ >= robots_position[j][5])
					|| (rMinZ >= robots_position[j][4] &&
										rMinZ <= robots_position[j][5])
					|| (rMaxZ >= robots_position[j][4] &&
										rMaxZ <= robots_position[j][5])) {
					inRangeZ = true;
			}

		}

		// Do not insert resources in the robot range
		//if (!(inRangeX && inRangeY && inRangeZ)) {
			environment_->addResource(resource);
		/**} else {
                    //TODO: Before removing the resource, try find a random
                    //position to place it at
                    resource->remove();
                    resourcesRemoved_ = true;
                    std::cout << "Box resource " << i << " removed" << std::endl;
                    resourceId--;
                    
		}*/

	}//end of resources loop
        
        //---------------------------------------------------------------------- 
        //Setup light sources
        //----------------------------------------------------------------------
	boost::shared_ptr<LightSourcesConfig> lightSourcesConfig =
			robogenConfig_->getLightSourcesConfig();

	std::vector<boost::shared_ptr<LightSource> > lightSources;

	std::vector<osg::Vec3> lightSourcesCoordinates =
			lightSourcesConfig->getCoordinates();
	std::vector<float> lightSourcesIntensities =
				lightSourcesConfig->getIntensities();

	for (unsigned int i = 0; i < lightSourcesCoordinates.size(); ++i) {
		double lMinX = lightSourcesCoordinates[i].x() - LightSource::RADIUS;
		double lMaxX = lightSourcesCoordinates[i].x() + LightSource::RADIUS;
		double lMinY = lightSourcesCoordinates[i].y() - LightSource::RADIUS;
		double lMaxY = lightSourcesCoordinates[i].y() + LightSource::RADIUS;
		double lMinZ = lightSourcesCoordinates[i].z() - LightSource::RADIUS;
		double lMaxZ = lightSourcesCoordinates[i].z() + LightSource::RADIUS;

		// Do not insert the ligh source if it is in the robot range
		bool inRangeX = false;
                bool inRangeY = false;
                bool inRangeZ = false;
                
                for (unsigned int j = 0 ; j < robots.size(); ++j){
                    if ((lMinX <= robots_position[j][0] && lMaxX >= robots_position[j][1]) 
                            || (lMinX >= robots_position[j][0] && lMinX <= robots_position[j][1])
                                    || (lMaxX >= robots_position[j][0] && lMaxX <= robots_position[j][1])) {
                            inRangeX = true;
                    }

                    if ((lMinY <= robots_position[j][2] && lMaxY >= robots_position[j][3]) 
                            || (lMinY >= robots_position[j][2] && lMinY <= robots_position[j][3])
                                    || (lMaxY >= robots_position[j][2] && lMaxY <= robots_position[j][3])) {
                            inRangeY = true;
                    }

                    if ((lMinZ <= robots_position[j][4] && lMaxZ >= robots_position[j][5]) 
                            || (lMinZ >= robots_position[j][4] && lMinZ <= robots_position[j][5])
                                    || (lMaxZ >= robots_position[j][4] && lMaxZ <= robots_position[j][5])) {
                            inRangeZ = true;
                    }
                }

		if (!(inRangeX && inRangeY && inRangeZ)) {
			lightSources.push_back(boost::shared_ptr<LightSource>(
						new LightSource(odeSpace, lightSourcesCoordinates[i],
								lightSourcesIntensities[i])));
		} else {
			if (robogenConfig_->getObstacleOverlapPolicy() ==
					RobogenConfig::ELEVATE_ROBOT) {

				if (lMaxZ > overlapMaxZ)
					overlapMaxZ = lMaxZ;
				lightSources.push_back(boost::shared_ptr<LightSource>(
						new LightSource(odeSpace, lightSourcesCoordinates[i],
								lightSourcesIntensities[i])));
			} else {
				obstaclesRemoved_ = true;
			}
		}



	}
	environment_->setLightSources(lightSources);



        for (unsigned int i = 0 ; i < robots.size(); ++i){
            if (robogenConfig_->getObstacleOverlapPolicy() ==
                            RobogenConfig::ELEVATE_ROBOT) {

                    robots[i]->translateRobot(
                                    osg::Vec3(starting_positions[i].x(), starting_positions[i].y(),
                                                    overlapMaxZ + inMm(2) - robots_position[i][4]));
            }

            // optimize the physics!  replace all fixed joints with composite bodies
            robots[i]->optimizePhysics();
        }
	return true;

    
}

boost::shared_ptr<StartPosition> Scenario::getCurrentStartPosition() {
	return robogenConfig_->getStartingPos()->getStartPosition(
			startPositionId_);
}

void Scenario::prune(){
	environment_.reset();
	robot_.reset();
}

//SM added
void Scenario::pruneswarm(){
    environment_.reset();
    for (unsigned int i = 0; i < robots_.size(); ++i ){
        robots_[i].reset();
    }
    robots_.clear();
}

boost::shared_ptr<Robot> Scenario::getRobot() {
	return robot_;
}
boost::shared_ptr<Robot> Scenario::getRobot(int robotId) {
	return robots_[robotId];
}
// SM added
std::vector<boost::shared_ptr<Robot> > Scenario::getRobots(){
    return robots_;
}

boost::shared_ptr<RobogenConfig> Scenario::getRobogenConfig() {
	return robogenConfig_;
}

void Scenario::setStartingPosition(int id) {
	startPositionId_ = id;
}

boost::shared_ptr<Environment> Scenario::getEnvironment() {
	return environment_;
}

dGeomID Scenario::getTerrainID() {
	return terrain_;
}


}
