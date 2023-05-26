/*
 * @(#) RobogenConfig.h   1.0   Mar 12, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Andrea Maesani, Joshua Auerbach
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
#ifndef ROBOGEN_ROBOGEN_CONFIG_H_
#define ROBOGEN_ROBOGEN_CONFIG_H_

#include <boost/shared_ptr.hpp>
#include "config/ObstaclesConfig.h"
#include "config/StartPositionConfig.h"
#include "config/TerrainConfig.h"
#include "config/LightSourcesConfig.h"
#include "config/GatheringZoneConfig.h"
#include "config/ResourcesConfig.h"
#include "robogen.pb.h"

namespace robogen {

/**
 * \brief Parameters for experimental scenarios supported by the simulator
 */
class RobogenConfig {

public:
	/**
	 * \brief Describes how to handle overlaps with obstacles
	 */
	enum ObstacleOverlapPolicies {
		REMOVE_OBSTACLES, CONSTRAINT_VIOLATION, ELEVATE_ROBOT
	};

	/**
	 * \brief Constructor.\n
	 * Initializes a robogen config object from configuration parameters
	 */
	RobogenConfig(std::string scenario, std::string scenarioFile,
			unsigned int timeSteps,
			float timeStepLength, int actuationPeriod,
			boost::shared_ptr<TerrainConfig> terrain,
			boost::shared_ptr<ObstaclesConfig> obstacles,
			std::string obstacleFile,
			boost::shared_ptr<StartPositionConfig> startPositions,
			std::string startPosFile,
			boost::shared_ptr<ResourcesConfig> resources,
			std::string resourceFile,
			boost::shared_ptr<LightSourcesConfig> lightSources,
			std::string lightSourceFile,
			float sensorNoiseLevel, float motorNoiseLevel,
			bool capAcceleration, float maxLinearAcceleration,
			float maxAngularAcceleration, int maxDirectionShiftsPerSecond,
			osg::Vec3 gravity, bool disallowObstacleCollisions,
			unsigned int obstacleOverlapPolicy,
            boost::shared_ptr<GatheringZoneConfig> gatheringZone,
			int swarmSize, std::string mode, bool complexityCost=false) :
				scenario_(scenario), scenarioFile_(scenarioFile),
				timeSteps_(timeSteps),
				timeStepLength_(timeStepLength),
				actuationPeriod_(actuationPeriod),
				terrain_(terrain), obstacles_(obstacles),
				obstacleFile_(obstacleFile),
				startPositions_(startPositions),
				startPosFile_(startPosFile),
				resources_(resources),
				resourceFile_(resourceFile),
				lightSources_(lightSources),
				lightSourceFile_(lightSourceFile),
				sensorNoiseLevel_(sensorNoiseLevel),
				motorNoiseLevel_(motorNoiseLevel),
				capAcceleration_(capAcceleration),
				maxLinearAcceleration_(maxLinearAcceleration),
				maxAngularAcceleration_(maxAngularAcceleration),
				maxDirectionShiftsPerSecond_(maxDirectionShiftsPerSecond),
				gravity_(gravity),
				disallowObstacleCollisions_(disallowObstacleCollisions),
				obstacleOverlapPolicy_(obstacleOverlapPolicy), 
				gatheringZone_(gatheringZone), swarmSize_(swarmSize),
				mode_(mode), complexityCost_(complexityCost) {

		simulationTime_ = timeSteps * timeStepLength;

	}

	/**
	 * Constructor\n
	 * Initializes a robogen config object from a message
	 */
	RobogenConfig(const robogenMessage::SimulatorConf &message);

	/**
	 * \brief Destructor
	 */
	virtual ~RobogenConfig() {

	}

	/**
	 * \brief Returns simulation scenario.\n
	 * @return the simulation scenario
	 */
	std::string getScenario() const {
		return scenario_;
	}

	/**
	 * \brief Returns scenario file.\n
	 * @return the simulation scenario file (for scripted scenario)
	 */
	std::string getScenarioFile() const {
		return scenarioFile_;
	}

	/**
	 * \brief Returns terrain config\n
	 * @return the terrain configuration
	 */
	boost::shared_ptr<TerrainConfig> getTerrainConfig() {
		return terrain_;
	}

	/**
	 * \brief Returns obstacles config\n
	 * @return the obstacles configuration
	 */
	boost::shared_ptr<ObstaclesConfig> getObstaclesConfig() {
		return obstacles_;
	}

	/**
	 * \brief Returns light sources config\n
	 * @return the light sources configuration
	 */
	boost::shared_ptr<LightSourcesConfig> getLightSourcesConfig() {
		return lightSources_;
	}

	/**
	 * \brief Returns timesteps\n
	 * @return the number of timesteps
	 */
	unsigned int getTimeSteps() const {
		return timeSteps_;
	}

	/**
	 * \brief Returns simulation time\n
	 * @return the total simulation time
	 */
	float getSimulationTime() const {
		return simulationTime_;
	}

	/**
	 * \brief Returns time step length\n
	 * @return the time step length
	 */
	float getTimeStepLength() const {
		return timeStepLength_;
	}

	/**
	 * \brief Returns actuation period\n
	 * @return the actuation period
	 */
	int getActuationPeriod() const {
		return actuationPeriod_;
	}

	/**
	 * \brief Returns robot starting positions\n
	 * @return the robot starting positions
	 */
	boost::shared_ptr<StartPositionConfig> getStartingPos() {
		return startPositions_;
	}

	/**
	 * \brief Returns starting positions file\n
	 * @return the starting position configuration file
	 */
	std::string getStartPosFile(){
		return startPosFile_;
	}

	/**
	 * \brief Returns obstacle config file\n
	 * @return the obstacle configuration file
	 */
	std::string getObstacleFile(){
		return obstacleFile_;
	}

	/**
	 *
	 * @return the light source configuration file
	 */
	std::string getLightSourceFile(){
		return lightSourceFile_;
	}

	/**
	 * \brief Returns sensors noise level\n
	 * Sensor noise is Gaussian with std dev of sensorNoiseLevel * actualValue
	 * i.e. value given to Neural Network is N(a, a * s)\n
	 * where a is actual value and s is sensorNoiseLevel
	 * @return sensor noise level
	 */
	float getSensorNoiseLevel() {
		return sensorNoiseLevel_;
	}

	/**
	 * \brief Return smotor noise level\n
	 * Motor noise is uniform in range +/- motorNoiseLevel * actualValue
	 * @return motor noise level
	 */
	float getMotorNoiseLevel() {
		return motorNoiseLevel_;
	}

	/**
	 * \brief Returns whether acceleration is capped or not\n
	 * @return if acceleration is capped
	 */
	bool isCapAlleration() {
		return capAcceleration_;
	}

	/**
	 * \brief Returns max linear acceleration\n
	 * @return max linear acceleration (if capped)
	 */
	float getMaxLinearAcceleration() {
		return maxLinearAcceleration_;
	}

	/**
	 * \brief Returns max angular acceleration.\n
	 * @return max angular acceleration (if capped)
	 */
	float getMaxAngularAcceleration() {
		return maxAngularAcceleration_;
	}

	/**
	 * \brief Returns max direction shift per second\n
	 * @return max direction shifts per second for testing motor burnout
	 * 		if -1, then do not check burnout
	 */
	int getMaxDirectionShiftsPerSecond() {
		return maxDirectionShiftsPerSecond_;
	}

	/**
	 * \brief Returns the value of gravity in each dimension\n
	 * @return gravity vector
	 */
	osg::Vec3 getGravity() {
		return gravity_;
	}

	/**
	 * \brief Returns whether obstacle collisions are allowed or not\n
	 * return if should disallow obstacle collisions
	 */
	bool isDisallowObstacleCollisions() {
		return disallowObstacleCollisions_;
	}

	/**
	 * \brief Returns method for handling obstacle overlaps.\n
	 * @return how to handle obstacle overlaps
	 */
	unsigned int getObstacleOverlapPolicy() {
		return obstacleOverlapPolicy_;
	}

	/**
	 * CH - Returns the complexity cost flag (1==true, 0==false)\n
	 * @return the complexity cost flag
	 */
	bool getComplexityCost(){
		return complexityCost_;
	}

	/**
	 * \brief Returns GatheringZone - SM\n
	 * @return the gathering zone configuration
	 */
	boost::shared_ptr<GatheringZoneConfig> getGatheringZoneConfig() {
		return gatheringZone_;
	}
        
	/**
	 * \brief Returns Resources config - SM\n
	 * @return the resources configuration
	 */
	boost::shared_ptr<ResourcesConfig> getResourcesConfig() {
		return resources_;
	}
        
	/**
	 * \brief Returns config file - SM\n
	 * @return the resource configuration file
	 */
	std::string getResourceFile(){
		return resourceFile_;
	}

	/**
	 * \brief Returns swarm size -SM\n
	 * @return the swarm size
	 */
	int getSwarmSize(){
		return swarmSize_;
	}
        
	/**
	 * \brief Returns swarm size - SM\n
	 * @return the swarm size
	 */
	std::string getMode(){
		return mode_;
	}
	/**
	 * \brief Convert configuration into configuration message.
	 */
	robogenMessage::SimulatorConf serialize() const{
		robogenMessage::SimulatorConf ret;

		ret.set_ntimesteps(timeSteps_);
		ret.set_scenario(scenario_);
		ret.set_timestep(timeStepLength_);
		ret.set_actuationperiod(actuationPeriod_);
		ret.set_sensornoiselevel(sensorNoiseLevel_);
		ret.set_motornoiselevel(motorNoiseLevel_);
		ret.set_capacceleration(capAcceleration_);
		ret.set_maxlinearacceleration(maxLinearAcceleration_);
		ret.set_maxangularacceleration(maxAngularAcceleration_);
		ret.set_maxdirectionshiftspersecond(maxDirectionShiftsPerSecond_);
		ret.set_gravityx(gravity_.x());
		ret.set_gravityy(gravity_.y());
		ret.set_gravityz(gravity_.z());
		ret.set_disallowobstaclecollisions(disallowObstacleCollisions_);
		ret.set_obstacleoverlappolicy(obstacleOverlapPolicy_);

		terrain_->serialize(ret);

		obstacles_->serialize(ret);
		startPositions_->serialize(ret);
		lightSources_->serialize(ret);
                
                gatheringZone_ ->serialize(ret);
		return ret;
	}

private:

	/**
	 * The simulation scenario
	 */
	std::string scenario_;

	/**
	 * The simulation scenario file (if using scripted scenario)
	 */
	std::string scenarioFile_;

	/**
	 * Total number of simulation timesteps
	 */
	unsigned int timeSteps_;

	/**
	 * Time step duration
	 */
	float timeStepLength_;

	/**
	 * Actuation period (in number of time steps)
	 */
	int actuationPeriod_;

	/**
	 * Terrain configuration
	 */
	boost::shared_ptr<TerrainConfig> terrain_;

	/**
	 * Obstacles configuration
	 */
	boost::shared_ptr<ObstaclesConfig> obstacles_;

	/**
	 * Obstacle configuration file location
	 */
	std::string obstacleFile_;

	/**
	 * List of robot starting positions
	 */
	boost::shared_ptr<StartPositionConfig> startPositions_;

	/**
	 * Starting positions configuration file location
	 */
	std::string startPosFile_;

	/**
	 * Light sources configuration
	 */
	boost::shared_ptr<LightSourcesConfig> lightSources_;

	/**
	 * Light sources configuration file location
	 */
	std::string lightSourceFile_;

	/**
	 * Simulation time
	 */
	float simulationTime_;

	/**
	 * Sensor noise level (see getter for details)
	 */
	float sensorNoiseLevel_;

	/**
	 * Motor noise level (see getter for details)
	 */
	float motorNoiseLevel_;


	/**
	 *  Flag to enforce acceleration cap.  Useful for preventing unrealistic
	 *  behaviors / simulator exploits
	 */
	bool capAcceleration_;

	/**
	 *  Maximum allowed linear acceleration if acceleration is capped
	 */
	float maxLinearAcceleration_;

	/**
	 *  Maximum allowed angular acceleration if acceleration is capped
	 */
	float maxAngularAcceleration_;

	/**
	 *  Maximum allowed direction shifts per second (if specified)
	 */
	int maxDirectionShiftsPerSecond_;

	/**
	 * Gravity
	 */
	osg::Vec3 gravity_;

	/**
	 * flag to disallow obstacle collisions
	 */
	bool disallowObstacleCollisions_;

	/**
	 * policy for handling the situation when an obstacle is in the robot's
	 * initial AABB
	 */
	unsigned int obstacleOverlapPolicy_;

	/**
	 * CH - flag to enable or disable complexity cost
	 */
	bool complexityCost_;
        
	/**
	 * SM - Gathering zone configuration
	 */
	boost::shared_ptr<GatheringZoneConfig> gatheringZone_;
        
	/**
	 * SM added - Resources configuration
	 */
	boost::shared_ptr<ResourcesConfig> resources_;
        
	/**
	 * SM added - Resource configuration file location
	 */
	std::string resourceFile_;

	/**
	 * SM added - Swarm size
	 */
	int swarmSize_;

	/**
	 * SM Added
	 * Simulation mode
	 */
	std::string mode_;

};

}

#endif /* ROBOGEN_ROBOGEN_CONFIG_H_ */
