/*
 * @(#) Scenario.h   1.0   Mar 13, 2013
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
#ifndef ROBOGEN_SCENARIO_H_
#define ROBOGEN_SCENARIO_H_

#include <boost/shared_ptr.hpp>
#include <vector>
#include "Robogen.h"
#include "config/StartPosition.h"


namespace robogen {

class BoxObstacle;
class Environment;
class RobogenConfig;
class Robot;
class Terrain;
class BoxResource;
/**
 * \brief Class to describe a Scenario for the Robogen simulator.
 */
class Scenario {

public:

	/**
	 * \brief AConstructor
	 *
	 *  scenario is not valid until the
	 * init() method is called successfully.
	 *
	 * @param robogenConfig
	 */
	Scenario(boost::shared_ptr<RobogenConfig> robogenConfig);

	/**
	 * \brief Destructor
	 */
	virtual ~Scenario();

	/**
	 * \brief Initializes a scenario
	 *
	 * @param odeWorld
	 * @param odeSpace
	 * @param robot
	 */
	virtual bool init(dWorldID odeWorld, dSpaceID odeSpace,
			boost::shared_ptr<Robot> robot);
	/**
	 * \brief Initializes a scenario with multiple robots - SM
	 *
	 * @param odeWorld
	 * @param odeSpace
	 * @param robot-swarm
	 */
	virtual bool init(dWorldID odeWorld, dSpaceID odeSpace, dSpaceID areaSpace,
						std::vector<boost::shared_ptr<Robot> > robots);

	/**
	 * \brief Clears unused scenario (so that robot can be freed - joints need to be
	 * destroyed before the destruction of the world).
	 *
	 * Undoes what init() does
	 * and avoids memory leaks over multiple starting positions.
	 */
	void prune();
	/**
	 * \brief Clears unused scenario (so that robots can be freed - joints need to be
	 * destroyed before the destruction of the world).
	 *
	 * Undoes what init() does
	 * and avoids memory leaks over multiple starting positions. SM
	 *
	 */
	void pruneswarm();

	/**
	 * \brief Returns robot
	 * @return the robot
	 */
	boost::shared_ptr<Robot> getRobot();
        
	/**
	 * \brief Returns specified robot
	 *
	 * @return the robot
	 */
	boost::shared_ptr<Robot> getRobot(int robotId);
        
	/**
	 * \brief Returns vector of robot swarm - SM
	 * @return the robots
	 */
	std::vector<boost::shared_ptr<Robot> > getRobots();
        
	/**
	 * \brief Returns Robogen config
	 *
	 * @return the robogen configuration
	 */
	boost::shared_ptr<RobogenConfig> getRobogenConfig();

	/**
	 * \brief Returns Environment
	 *
	 * @return the environment
	 */
	boost::shared_ptr<Environment> getEnvironment();

	/**
	 * \brief Returns Terrain Id
	 * @return Terrain id
	 */
	dGeomID getTerrainID();
	/**
	 * Returns whether obstacles were removed
	 */
	inline bool wereObstaclesRemoved() {
		return obstaclesRemoved_;
	}

	/**
	 * \brief Sets the starting position id
	 */
	void setStartingPosition(int id);

	/**
	 * \brief Setup the simulation scenario. Called before the physics simulation begins.
	 * @return true if the operation completed successfully, false otherwise
	 */
	virtual bool setupSimulation() = 0;

	/**
	 * \brief Called after each simulation step
	 * @return true if the operation completed successfully, false otherwise
	 */
	virtual bool afterSimulationStep() = 0;

	/**
	 * \brief This is called just after afterSimulationStep
	 * @return stopSimulationNow_, which is true only if the scenario has
	 * 	called stopSimulationNow()
	 */
	bool shouldStopSimulationNow() { return stopSimulationNow_; }

	/**
	 * \brief Called at the end of the physics simulation
	 * @return true if the operation completed successfully, false otherwise
	 */
	virtual bool endSimulation() = 0;

	/**
	 * \brief Compute the fitness
	 * @return fitness
	 */
	virtual double getFitness() = 0;

	/**
	 * \brief BK-added - Get the endPosition
	 * @return endPosition
	 */
	virtual std::vector<float> getEndPosition() = 0;

	/**
	 * \brief Returns whether there are still remaining trials
	 * @return true if another trial must be executed
	 */
	virtual bool remainingTrials() = 0;

	/**
	 * \brief Returns current trial
	 *
	 * @return the current trial
	 */
	virtual int getCurTrial() const = 0;

	/**
	 * \brief Returns current trial starting position
	 *
	 * @return the current trial starting position
	 */
	boost::shared_ptr<StartPosition> getCurrentStartPosition();
	/**
	 * \brief Sets Robogen config
	 */
	void setRobogenConfig(boost::shared_ptr<RobogenConfig> robogenConfig) {
		robogenConfig_ = robogenConfig;
	}

	/**
	 * \brief calling this will stop the simulation immediately after the next call to
	 * afterSimulationStep
	 */
	void stopSimulationNow() {
		stopSimulationNow_ = true;
	}
        
	/**
	 * \brief Returns whether resources were removed or not
	 *
	 * @return true if resources were removed, false otherwise
	 */
	inline bool wereResourcesRemoved() {
	return resourcesRemoved_;
	}
private:

	/**
	 * Robot
	 */
	boost::shared_ptr<Robot> robot_;

	/**
	 * Robogen config
	 */
	boost::shared_ptr<RobogenConfig> robogenConfig_;

	/**
	 * The current id of starting position
	 */
	int startPositionId_;

	/**
	 * The environment
	 */
	boost::shared_ptr<Environment> environment_;

	bool obstaclesRemoved_;

	bool stopSimulationNow_;
	//----------------------------------------------------------------------
	//SM added: swarm specific attributes
	//----------------------------------------------------------------------
	dGeomID terrain_;
	/**
	 * Robots
	 */
	std::vector<boost::shared_ptr<Robot> > robots_;
        bool resourcesRemoved_;
};

}

#endif /* ROBOGEN_SCENARIO_H_ */
