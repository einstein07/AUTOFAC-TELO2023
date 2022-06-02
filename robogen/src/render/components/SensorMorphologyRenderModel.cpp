/*
 * @(#) SensorMorphologyRenderModel.cpp   1.0   Jun 01, 2022
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#include <osgDB/ReadFile>

#include "render/callback/BodyCallback.h"
#include "render/components/SensorMorphologyRenderModel.h"
#include "render/Mesh.h"

#include "utils/RobogenUtils.h"

namespace robogen {

SensorMorphologyRenderModel::SensorMorphologyRenderModel(boost::shared_ptr<SensorMorphologyModel>
		model) : RenderModel(model) {
	this->partA_.reset(new Mesh());
}

SensorMorphologyRenderModel::~SensorMorphologyRenderModel() {

}

bool SensorMorphologyRenderModel::initRenderModel() {

	bool meshLoadingA  = this->partA_->loadMesh(
				RobogenUtils::getMeshFile(this->getModel(),
						SensorMorphologyModel::B_SENSOR_BASE_ID));
	if (!meshLoadingA) {
		std::cerr << "[SensorMorphologyRenderModel] Error loading model"
				<< std::endl;
		return false;
	}

	if (isDebugActive()) {
		this->showDebugView();
	}

	// PART A
	osg::ref_ptr<osg::PositionAttitudeTransform> partA =
			this->partA_->getMesh();
	partA_->setColor(osg::Vec4(0, 0, 1, 0.5));

	partA->setPosition(RobogenUtils::getRelativePosition(this->getModel(),
			SensorMorphologyModel::B_SENSOR_BASE_ID));

	partA->setAttitude(RobogenUtils::getRelativeAttitude(this->getModel(),
			SensorMorphologyModel::B_SENSOR_BASE_ID));

	osg::ref_ptr<osg::PositionAttitudeTransform> patPartA(
			new osg::PositionAttitudeTransform());
	patPartA->addChild(partA);

	this->getMeshes()->addChild(patPartA.get());
	patPartA->setUpdateCallback(
			new BodyCallback(this->getModel(),
					SensorMorphologyModel::B_SENSOR_BASE_ID));

	if (isDebugActive()) {
		this->activateTransparency(patPartA->getOrCreateStateSet());
	}

	return true;

}

void SensorMorphologyRenderModel::showDebugView() {

	this->attachGeoms();
}

void SensorMorphologyRenderModel::setColor(osg::Vec4 color) {
	this->partA_->setColor(color);
}

}


