/*
 * @(#) environment.h   1.0   Mar 6, 2013
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
#ifndef ROBOGEN_ENVIRONMENT_H_
#define ROBOGEN_ENVIRONMENT_H_

#include "model/objects/LightSource.h"
#include "model/objects/Obstacle.h"
#include "Terrain.h"
#include "config/RobogenConfig.h"
#include "model/objects/GatheringZone.h"
#include "model/objects/BoxResource.h"

#define DEFAULT_AMBIENT_LIGHT (0.04)

namespace robogen {


/**
 * \brief Class to describe simulation Environment
 *
 * Environment contains all info about the simulation except for the robot
 */

class Environment {

public:
	/**
	 * \brief Constructor
	 */
	Environment(dWorldID odeWorld, dSpaceID odeSpace,
			boost::shared_ptr<RobogenConfig> robogenConfig);
	/**
	 * \brief Constructor - SM
	 */
	Environment(dWorldID odeWorld, dSpaceID odeSpace, dSpaceID areaSpace,
				boost::shared_ptr<RobogenConfig> robogenConfig);
	/**
	 * \brief Initializes Environment
	 */
	bool init();
	/**
	 * \brief Destructor
	 */
	virtual ~Environment();
	/**
	 * \brief Sets time elapsed
	 */
	void setTimeElapsed(float timeElapsed) {
		timeElapsed_ = timeElapsed;
	}
	/**
	 * \brief Returns time elapsed
	 */
	float getTimeElapsed() {
		return timeElapsed_;
	}
	/**
	 * \brief Sets light sources
	 */
	void setLightSources(
			const std::vector<boost::shared_ptr<LightSource> >& lightSources) {
		lightSources_ = lightSources;
	}
	/**
	 * \brief Returns light sources
	 */
	const std::vector<boost::shared_ptr<LightSource> >& getLightSources() {
		return lightSources_;
	}
	/**
	 * \brief Set AAmbient light
	 */
	void setAmbientLight(float ambientLight) {
		ambientLight_ = ambientLight;
	}
	/**
	 * \brief Returns Ambient light
	 */
	float getAmbientLight() {
		return ambientLight_;
	}
	/**
	 * \brief Adds obstacle
	 */
	void addObstacle(boost::shared_ptr<Obstacle> obstacle) {
		obstacles_.push_back(obstacle);
	}
	/**
	 * \brief Returns Terrain Id
	 */
	dGeomID getTerrainID() {
		return terrain_ -> getGeom();
	}
	/**
	 * \brief Returns Terrain
	 */
	boost::shared_ptr<Terrain> getTerrain() {
		return terrain_ ;
	}
	/**
	 * \brief Returns Obstacles
	 */
	std::vector<boost::shared_ptr<Obstacle> > getObstacles() {
		return obstacles_;
	}

	/**
	 * \brief Sets GatheringZone - SM
	 * @arg the gathering zone
	 */
	void setGatheringZone(boost::shared_ptr<GatheringZone> gatheringZone) {
		gatheringZone_ = gatheringZone;
	}

	/**
	 * \brief Returns GatheringZone - SM
	 * @return the gathering zone
	 */
	boost::shared_ptr<GatheringZone> getGatheringZone() {
		return gatheringZone_;
	}

	/**
	 * \brief Step the gathering zone - SM
	 */
	void stepGatheringZone(std::vector<boost::shared_ptr<Robot> > robots, boost::mutex& queueMutex);

	/**
	 * \brief Returns resources - SM
	 * @return all resources in the environment
	 */
	std::vector<boost::shared_ptr<BoxResource> > getResources() {
		return resources_;
	}
        
        /**
         * \brief Adds resource - SM
         * @param resource the resource that is being added 
         */
        void addResource(boost::shared_ptr<BoxResource> resource) {
		resources_.push_back(resource);
	}

private:
	/**
	 * ODE physics world
	 */
	dWorldID odeWorld_;

	/**
	 * ODE collision space
	 */
	dSpaceID odeSpace_;
	/**
	 * ODE collision space
	 */
	dSpaceID areaSpace_;

	/**
	 * Robogen config
	 */
	boost::shared_ptr<RobogenConfig> robogenConfig_;

	/**
	 * Time elapsed since the last "refresh" of the environment
	 */
	float timeElapsed_;

	/**
	 * Light sources in the environment
	 */
	std::vector<boost::shared_ptr<LightSource> > lightSources_;

	/**
	 * Ambient light
	 */
	float ambientLight_;

	/**
	 * Terrain
	 */
	boost::shared_ptr<Terrain> terrain_;

	/**
	 * Obstacle in the environment
	 */

	std::vector<boost::shared_ptr<Obstacle> > obstacles_;
        
	/**
	 * SM - Added. Gathering zone
	 */
	boost::shared_ptr<GatheringZone> gatheringZone_;
        
	/**
	 * SM - Added. Resources in the environment
	 */

	std::vector<boost::shared_ptr<BoxResource> > resources_;

};

}

#endif /* ROBOGEN_ENVIRONMENT_H_ */
