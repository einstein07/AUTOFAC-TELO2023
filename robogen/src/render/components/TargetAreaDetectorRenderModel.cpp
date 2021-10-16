/*
 * @(#) TargetAreaDetectorRenderModel.cpp   1.0   Oct 16, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#include <osgDB/ReadFile>

#include "render/callback/BodyCallback.h"
#include "render/components/TargetAreaDetectorRenderModel.h"
#include "render/Mesh.h"

#include "utils/RobogenUtils.h"

namespace robogen {

TargetAreaDetectorRenderModel::TargetAreaDetectorRenderModel(boost::shared_ptr<TargetAreaDetectorModel>
		model) : RenderModel(model) {
	this->partA_.reset(new Mesh());
}

TargetAreaDetectorRenderModel::~TargetAreaDetectorRenderModel() {

}

bool TargetAreaDetectorRenderModel::initRenderModel() {

	bool meshLoadingA  = this->partA_->loadMesh(
				RobogenUtils::getMeshFile(this->getModel(),
						ColorSensorModel::B_SENSOR_BASE_ID));
	if (!meshLoadingA) {
		std::cerr << "[TargetAreaDetectorRenderModel] Error loading model"
				<< std::endl;
		return false;
	}

	if (isDebugActive()) {
		this->showDebugView();
	}

	// PART A
	osg::ref_ptr<osg::PositionAttitudeTransform> partA =
			this->partA_->getMesh();
	partA_->setColor(osg::Vec4(0, 1, 1, 0.5));

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

void TargetAreaDetectorRenderModel::showDebugView() {

	this->attachGeoms();
}

void TargetAreaDetectorRenderModel::setColor(osg::Vec4 color) {
	this->partA_->setColor(color);
}

}


