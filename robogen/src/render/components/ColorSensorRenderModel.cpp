/*
 * @(#) ColorSensorRenderModel.cpp   1.0   Aug 22, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#include <osgDB/ReadFile>

#include "render/callback/BodyCallback.h"
#include "render/components/ColorSensorRenderModel.h"
#include "render/Mesh.h"

#include "utils/RobogenUtils.h"

namespace robogen {

ColorSensorRenderModel::ColorSensorRenderModel(boost::shared_ptr<ColorSensorModel>
		model) : RenderModel(model) {
	this->partA_.reset(new Mesh());
}

ColorSensorRenderModel::~ColorSensorRenderModel() {

}

bool ColorSensorRenderModel::initRenderModel() {

	bool meshLoadingA  = this->partA_->loadMesh(
				RobogenUtils::getMeshFile(this->getModel(),
						ColorSensorModel::B_SENSOR_BASE_ID));

	if (!meshLoadingA) {
		std::cerr << "[ColorSensorRenderModel] Error loading model"
				<< std::endl;
		return false;
	}

	if (isDebugActive()) {
		this->showDebugView();
	}

	// PART A
	osg::ref_ptr<osg::PositionAttitudeTransform> partA =
			this->partA_->getMesh();

	partA_->setColor(osg::Vec4(1, 0, 0, 0.5));

	partA->setPosition(RobogenUtils::getRelativePosition(this->getModel(),
						ColorSensorModel::B_SENSOR_BASE_ID));

	partA->setAttitude(RobogenUtils::getRelativeAttitude(this->getModel(),
						ColorSensorModel::B_SENSOR_BASE_ID));


	osg::ref_ptr<osg::PositionAttitudeTransform> patPartA(
			new osg::PositionAttitudeTransform());
	patPartA->addChild(partA);

	this->getMeshes()->addChild(patPartA.get());
	patPartA->setUpdateCallback(
			new BodyCallback(this->getModel(),
					ColorSensorModel::B_SENSOR_BASE_ID));

	if (isDebugActive()) {
		this->activateTransparency(patPartA->getOrCreateStateSet());
	}

	return true;

}

void ColorSensorRenderModel::showDebugView() {

	this->attachGeoms();
}

void ColorSensorRenderModel::setColor(osg::Vec4 color) {
	this->partA_->setColor(color);
}

}


