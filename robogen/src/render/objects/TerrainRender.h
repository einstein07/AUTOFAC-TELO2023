/*
 * @(#) TerrainRender.h   1.0   Mar 12, 2013
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
#ifndef ROBOGEN_TERRAIN_RENDER_MODEL_H_
#define ROBOGEN_TERRAIN_RENDER_MODEL_H_

#include <boost/shared_ptr.hpp>
#include <osg/PositionAttitudeTransform>

namespace robogen {

class Terrain;
/**
 * \brief Class to render Terrain model
 */
class TerrainRender {

public:
	/**
	 * \brief Constructor
	 */
	TerrainRender(boost::shared_ptr<Terrain> terrain);
	/**
	 * \brief Destructor
	 */
	virtual ~TerrainRender();
	/**
	 * \brief Returns root node
	 */
	osg::ref_ptr<osg::PositionAttitudeTransform> getRootNode();

private:

	osg::ref_ptr<osg::PositionAttitudeTransform> rootNode_;


};


}


#endif /* ROBOGEN_TERRAIN_RENDER_MODEL_H_ */
