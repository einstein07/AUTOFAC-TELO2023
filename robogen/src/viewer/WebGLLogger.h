/*
 * @(#) FileViewerLog.h   1.0   Aug 20, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 * Guillaume Leclerc (guillaume.leclerc@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Titus Cieslweski, Joshua Auerbach
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

#ifndef WEBGLLOGGER_H_
#define WEBGLLOGGER_H_

#define JSON_TAGS JSON_REAL_PRECISION(10) | JSON_COMPACT

#include <boost/shared_ptr.hpp>
#include "Robot.h"
#include <string>
#include <fstream>
#include <scenario/Scenario.h>
#include <jansson.h>
#include <model/components/ParametricBrickModel.h>

namespace robogen {
/**
 * \brief Struct to describe BodyDescriptor
 */
struct BodyDescriptor {
	/**
	 * \brief Model
	 */
	boost::shared_ptr<Model> model;
	/**
	 * \brief Mesh name
	 */
	std::string meshName;
	/**
	 * \brief Body id
	 */
	int bodyId;
};
/**
 * \brief Class to describe WebGLLogger
 */
class WebGLLogger {
public:
	/**
	 * \brief Constructor
	 */
	WebGLLogger(std::string inFileName, boost::shared_ptr<Scenario> in_scenario,
			double targetFramerate = 120.0);
	/**
	 * \brief Log
	 */
	void log(double dt);
	/**
	 * \brief Destructor
	 */
	~ WebGLLogger();
	/**
	 * \brief Structure tag
	 */
	static const char* STRUCTURE_TAG;
	/**
	 * \brief Log tag
	 */
	static const char* LOG_TAG;
	/**
	 * \brief Attitude tag
	 */
	static const char* ATTITUDE_TAG;
	/**
	 * \brief Position tag
	 */
	static const char* POSITION_TAG;
	/**
	 * \brief REL Position tag
	 */
	static const char* REL_POS_TAG;
	/**
	 * \brief REL attitude tag
	 */
	static const char* REL_ATT_TAG;
	/**
	 * \brief Map tag
	 */
	static const char* MAP_TAG;
	/**
	 * \brief Mesh path
	 */
	static const char* MESH_PATH;
	/**
	 * \brief Map dimension tag
	 */
	static const char* MAP_DIM_TAG;
	/**
	 * \brief Map data tag
	 */
	static const char* MAP_DATA_TAG;
	/**
	 * \brief Obstacle tags
	 */
	static const char* OBSTACLE_TAGS;
	/**
	 * \brief Obstacle DEF tag
	 */
	static const char* OBSTACLE_DEF_TAG;
	/**
	 * \brief Obstacle LOG tag
	 */
	static const char* OBSTACLE_LOG_TAG;
	/**
	 * \brief Light tags
	 */
	static const char* LIGHT_TAGS;
	/**
	 * \brief Returns structure JSON
	 */
	std::string getStructureJSON();
	/**
	 * \brief Returns obstacle definition JSON
	 */
	std::string getObstaclesDefinitionJSON();
	/**
	 * \brief Returns lights JSON
	 */
	std::string getLightsJSON();
	/**
	 * \brief Returns last log JSON
	 */
	std::string getLastLogJSON();

private:
	double frameRate;
	double lastFrame;
	boost::shared_ptr<Robot> robot;
	boost::shared_ptr<Scenario> scenario;
	std::string fileName;
	json_t *jsonMap;
	json_t *jsonRoot;
	json_t *jsonStructure;
	json_t *jsonLog;
	json_t *jsonObstacles;
	json_t *jsonObstaclesDefinition;
	json_t *jsonObstaclesLog;
	json_t *jsonLights;

	std::vector<struct BodyDescriptor> bodies;

	static std::string getFormatedStringForCuboid(double width, double height,
			double thickness);
	static std::string getFormatedStringForCylinder(double radius,
			double height);

	//disable copy constructor;
	WebGLLogger(const WebGLLogger& that);
	//disable copy constructor
	const WebGLLogger& operator=(const WebGLLogger& that);
	void generateBodyCollection();
	void generateMapInfo();
	void writeJSONHeaders();
	void writeRobotStructure();
	void writeObstaclesDefinition();
};
}

#endif

