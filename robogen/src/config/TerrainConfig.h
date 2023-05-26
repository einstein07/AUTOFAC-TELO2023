/*
 * @(#) TerrainConfig.h   1.0   Mar 12, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani, Joshua Auerbach
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
#ifndef ROBOGEN_TERRAIN_CONFIG_H_
#define ROBOGEN_TERRAIN_CONFIG_H_

#include <string>
#include "robogen.pb.h"

namespace robogen {

/**
 * \brief Terrain configuration parameters
 */
class TerrainConfig {

public:

	enum TerrainType {
			EMPTY,
			FLAT,
			ROUGH
	};

	/**
	 * \brief Constructor.\nInitializes an empty terrain
	 *
	 * @param friction
	 */
	TerrainConfig(float friction) :
			type_(EMPTY), length_(0), width_(0), height_(0),
			friction_(friction) {

	}


	/**
	 * \brief Constructor\n
	 * Initializes a flat terrain
	 *
	 * @param length
	 * @param width
	 * @param friction
	 */
	TerrainConfig(float length, float width, float friction) :
			type_(FLAT), length_(length), width_(width), height_(0),
			friction_(friction) {

	}

	/**
	 * \brief Constructor\n
	 * Initializes a rough terrain
	 *
	 * @param heightFieldFileName
	 * @param length
	 * @param width
	 * @param height
	 */
	TerrainConfig(const std::string& heightFieldFileName, float length,
			float width, float height, float friction) :
				type_(ROUGH), heightFieldFileName_(heightFieldFileName),
				length_(length), width_(width), height_(height),
				friction_(friction) {

	}

	/**
	 * \brief Destructor
	 */
	virtual ~TerrainConfig() {

	}

	/**
	 * \brief Returns terrain type
	 * @return true if the terrain is flat, false otherwise
	 */
	TerrainType getType() {
		return type_;
	}

	/**
	 * \brief Returns heightfield file
	 * @return the heightfield file name
	 */
	const std::string& getHeightFieldFileName() const {
		return heightFieldFileName_;
	}

	/**
	 * \brief Returns terrain length
	 * @return the length of the terrain
	 */
	float getLength() {
		return length_;
	}

	/**
	 * \brief Returns terrain width
	 * @return the width of the terrain
	 */
	float getWidth() {
		return width_;
	}

	/**
	 * \brief Returns terrain height
	 * @return the height of the terrain
	 */
	float getHeight() {
		return height_;
	}

	/**
	 * \brief Returns frisction coefficient
	 * TODO: make this be more configurable than just global
	 * 	friction coefficient
	 * @return the friction coefficient of the terrain
	 *
	 */
	float getFriction() {
		return friction_;
	}

	/**
	 * \brief Serializes terrain details into a SimulatorConf message
	 */
	void serialize(robogenMessage::SimulatorConf &message) {
		message.set_terraintype(type_);
		message.set_terrainheightfieldfilename(heightFieldFileName_);
		message.set_terrainlength(length_);
		message.set_terrainwidth(width_);
		message.set_terrainheight(height_);
		message.set_terrainfriction(friction_);
	}

private:

	/**
	 * type of terrain
	 */
	TerrainType type_;

	/**
	 * If terrain is rugged, contains the height field file name
	 */
	std::string heightFieldFileName_;

	/**
	 * Terrain length
	 */
	float length_;

	/**
	 * Terrain width
	 */
	float width_;

	/**
	 * Terrain maximum height
	 */
	float height_;

	/**
	 * Terrain friction coefficient
	 */
	float friction_;

};

}

#endif /* ROBOGEN_TERRAIN_CONFIG_H_ */
