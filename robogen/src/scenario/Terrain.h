/*
 * @(#) Terrain.h   1.0   Mar 12, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
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
#ifndef ROBOGEN_TERRAIN_H_
#define ROBOGEN_TERRAIN_H_

#include <osg/ref_ptr>
#include <string>
#include "Robogen.h"
#include "config/TerrainConfig.h"

namespace osg {
class Image;
}

namespace robogen {

/**
 * \brief Class to describe Terrain model.
 *
 * The terrain can be either a flat surface or defined by an height map
 * that is tiled infinitely or totally empty.
 */
class Terrain {

public:

	/**
	 * \brief Constructor
	 *
	 * Initializes the model for terrain
	 */
	Terrain(dWorldID odeWorld, dSpaceID odeSpace);

	/**
	 * \brief Destructor
	 */
	virtual ~Terrain();

	/**
	 * \brief Returns Terrain type
	 * @return type of terrain
	 */
	TerrainConfig::TerrainType getType();

	/**
	 * \brief Initializes a flat terrain
	 * @param width
	 * @param depth
	 */
	bool initFlat(float width, float depth);

#ifndef DISABLE_HEIGHT_MAP
	/**
	 * \brief Initializes a rough terrain
	 *
	 * @param heightMapFileName the height map file defining terrain elevation contains a nxm matrix of byte values (0-255).
	 *                          0 Corresponds to the lower elevation, 1 to the maximum elevation
	 * @param width
	 * @param depth
	 * @param height
	 */
	bool initRough(const std::string& heightMapFileName, float width,
			float depth, float height);
#endif

	/**
	 * \brief Returns heightfield data
	 *
	 * @return the heightfield data
	 */
	osg::ref_ptr<osg::Image> getHeightFieldData();

	/**
	 * \brief Returns terrain width
	 *
	 * @return width
	 */
	float getWidth() const;

	/**
	 * \brief Returns heightfield depth
	 *
	 * @return heightfield depth
	 */
	float getDepth() const;

	/**
	 * \brief Returns heightfield height
	 *
	 * @return heightfield height
	 */
	float getHeightFieldHeight() const;

	/**
	 * \brief Returns Axis-Bounding-Box
	 *
	 * @return the AABB for the terrain (useful if the terrain is an heightfield)
	 */
	void getBB(double& minX, double& maxX, double& minY, double& maxY,
			double& minZ, double& maxZ);

	/**
	 * \brief Returns Geom - SM
	 */
	inline const dGeomID& getGeom() {
		return tID;
	}
private:

	/**
	 * ODE World
	 */
	dWorldID odeWorld_;

	/**
	 * ODE Geometry collision space
	 */
	dSpaceID odeSpace_;

	/**
	 * Type of terrain
	 */
	TerrainConfig::TerrainType type_;

	/**
	 * The heightfield
	 */
	dHeightfieldDataID heightField_;

	/**
	 * Height field data
	 */
	osg::ref_ptr<osg::Image> heightFieldData_;

	/**
	 * Height field depth, width and height
	 */
	float heightFieldWidth_;
	float heightFieldDepth_;
	float heightFieldHeight_;

	/**
	 * The ODE geometry associated with the terrain
	 */
	dGeomID odeGeometry_;

	/**
	 * SM Added
	 */
	dGeomID tID;

};

}

#endif /* ROBOGEN_TERRAIN_H_ */
