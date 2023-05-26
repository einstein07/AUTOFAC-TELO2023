/*
 * @(#) RenderModel.h   1.0   Feb 5, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
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
#ifndef ROBOGEN_RENDER_MODEL_H_
#define ROBOGEN_RENDER_MODEL_H_

#include <boost/weak_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include <osg/Node>
#include <osg/PositionAttitudeTransform>

namespace robogen {

class Model;
/**
 * \brief Class to render a model
 */
class RenderModel {

public:
	/**
	 * \brief Constructor
	 */
	RenderModel(boost::shared_ptr<Model> model);
	/**
	 * \brief Destructor
	 */
	virtual ~RenderModel();
	/**
	 * \brief Returns a root node
	 */
	osg::ref_ptr<osg::PositionAttitudeTransform> getRootNode();
	/**
	 * \brief Initializes a render model
	 */
	virtual bool initRenderModel() = 0;
	/**
	 * \brief Attaches axis to model
	 */
	static void attachAxis(osg::Transform* transform);
	/**
	 * \brief Returns debug status
	 */
	bool isDebugActive();
	/**
	 * \brief Returns box
	 */
	osg::ref_ptr<osg::Geode> getBox(float lengthX, float lengthY,
			float lengthZ);
	/**
	 * \brief Returns box
	 */
	osg::ref_ptr<osg::Geode> getBox(float lengthX, float lengthY,
				float lengthZ, const osg::Vec4& color);
	/**
	 * \brief Returns cylinder
	 */
	osg::ref_ptr<osg::Geode> getCylinder(float radius, float height,
			const osg::Vec4& color);
	/**
	 * \brief Returns capsule
	 */
	osg::ref_ptr<osg::Geode> getCapsule(float radius, float height);
	/**
	 * \brief Returns model
	 */
	boost::shared_ptr<Model> getModel();
	/**
	 * \brief Sets model color
	 */
	virtual void setColor(osg::Vec4 color) = 0;
	/**
	 * \brief Toggles debug status
	 */
	void setDebugActive(bool debugActive);
	/**
	 * \brief Toggles primitives
	 */
	void togglePrimitives(bool primitives);
	/**
	 * \brief Toggles meshes
	 */
	void toggleMeshes(bool meshes);

	//void toggleTransparency(bool transparency);

protected:

	/**
	 * \brief Attaches box
	 *
	 * Measures are expressed in mm
	 */
	osg::ref_ptr<osg::PositionAttitudeTransform> attachBox(int label, float lengthX, float lengthY, float lengthZ, const osg::Vec4& color);
	/**
	 * \brief Attaches box
	 *
	 * Measures are expressed in mm
	 */
	osg::ref_ptr<osg::PositionAttitudeTransform> attachBox(int label, float lengthX, float lengthY, float lengthZ);
	/**
	 * \brief Attaches Geode
	 */
	osg::ref_ptr<osg::PositionAttitudeTransform> attachGeode(int label, osg::ref_ptr<osg::Geode> geode);
	/**
	 * \brief Attaches Geoms	 */
	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > attachGeoms();
	/**
	 * \brief Attaches Geoms
	 */
	std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > attachGeoms(std::vector<osg::Vec4> colors);
	/**
	 * \brief Activates transparency
	 */
	void activateTransparency(osg::StateSet*);
	/**
	 * \brief Returns meshes
	 */
	inline osg::ref_ptr<osg::Group> getMeshes() {
		return meshes_;
	}

private:

	/**
	 * The root Position Attitude Transform defines the relative position of
	 * the component with respect to the parent component
	 */
	osg::ref_ptr<osg::PositionAttitudeTransform> rootNode_;

	/**
	 * The physics model
	 * keep as weak ptr so does not mess up ode clean up
	 */
	boost::weak_ptr<Model> model_;

	/**
	 * Debug mode
	 */
	bool debugActive_;

	osg::ref_ptr<osg::Group> primitives_;
	osg::ref_ptr<osg::Group> meshes_;

};

}

#endif /* ROBOGEN_RENDER_MODEL_H_ */
