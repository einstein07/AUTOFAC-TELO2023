/*
 * @(#) BoxResourceRender.h   1.0   August 02, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/BlendFunc>

#include "model/objects/BoxResource.h"

#include "render/callback/PositionObservableCallback.h"
#include "render/objects/BoxResourceRender.h"

namespace robogen {

BoxResourceRender::BoxResourceRender(boost::shared_ptr<BoxResource> resource): resource_(resource), debugActive_(false) {

	this->rootNode_ = osg::ref_ptr<osg::PositionAttitudeTransform>(
				new osg::PositionAttitudeTransform());

}

BoxResourceRender::~BoxResourceRender() {

}

bool BoxResourceRender::initRenderModel(){
	osg::ref_ptr<osg::Box> box(
			new osg::Box(osg::Vec3(0, 0, 0), fromOde(resource_->getGeomSize().x()),
					fromOde(resource_->getGeomSize().y()), fromOde(resource_->getGeomSize().z())));

	osg::ref_ptr<osg::ShapeDrawable> boxDrawable(
			new osg::ShapeDrawable(box.get()));
	boxDrawable->setColor(osg::Vec4(0, 1, 0, 1));
	osg::ref_ptr<osg::Geode> geode(new osg::Geode());
	geode->addDrawable(boxDrawable.get());
	rootNode_->addChild(geode);
	rootNode_->setUpdateCallback(new PositionObservableCallback(resource_));
	if (isDebugActive()) {
		this->showDebugView();
	}
	return true;

}

osg::ref_ptr<osg::PositionAttitudeTransform> BoxResourceRender::getRootNode() {
	return rootNode_;
}

bool BoxResourceRender::isDebugActive() {
	return debugActive_;
}

void BoxResourceRender::setDebugActive(bool debugActive) {
	this->debugActive_ = debugActive;
}

void BoxResourceRender::attachAxis(osg::Transform* transform) {

	std::cout << "Displaying axis: x=red, y=green, z=blue" << std::endl;

	osg::TessellationHints* hints = new osg::TessellationHints;

	double height = 100;
	double radius = .5;

	osg::MatrixTransform* zmt = new osg::MatrixTransform();

	transform->addChild(zmt);
	osg::ShapeDrawable *zShape = new osg::ShapeDrawable(
			new osg::Cylinder(osg::Vec3(0.0f, 0.0f, height / 2), radius,
					height), hints);
	osg::ShapeDrawable *zCone = new osg::ShapeDrawable(
			new osg::Cone(osg::Vec3(0.0f, 0.0f, 1.0), radius + 1.0, 2.0),
			hints);

	osg::MatrixTransform* zmtCone = new osg::MatrixTransform();
	osg::Geode *zgCone = new osg::Geode;

	zmtCone->setMatrix(osg::Matrix::translate(0.0, 0.0, height));
	transform->addChild(zmtCone);

	zShape->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
	zCone->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
	osg::Geode *z = new osg::Geode;
	z->addDrawable(zShape);
	zgCone->addDrawable(zCone);
	zmtCone->addChild(zgCone);
	zmt->addChild(z);

	osg::MatrixTransform* mt = new osg::MatrixTransform();
	transform->addChild(mt);

	osg::Matrix xMatrix = osg::Matrix::rotate(-osg::PI_2, 0.0, 1.0, 0.0);
	mt->setMatrix(xMatrix);

	osg::ShapeDrawable *xShape = new osg::ShapeDrawable(
			new osg::Cylinder(osg::Vec3(0.0f, 0.0f, height / 2), radius,
					height), hints);
	xShape->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
	osg::Geode *x = new osg::Geode;
	x->addDrawable(xShape);
	mt->addChild(x);

	osg::MatrixTransform *yMt = new osg::MatrixTransform();
	transform->addChild(yMt);
	osg::Matrix yMatrix = osg::Matrix::rotate(osg::PI_2, 1.0, 0.0, 0.0);
	yMt->setMatrix(yMatrix);

	osg::ShapeDrawable *yShape = new osg::ShapeDrawable(
			new osg::Cylinder(osg::Vec3(0.0f, 0.0f, height / 2), radius,
					height), hints);
	yShape->setColor(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
	osg::Geode *y = new osg::Geode;
	y->addDrawable(yShape);
	yMt->addChild(y);
}

void BoxResourceRender::showDebugView() {

		attachAxis(rootNode_);

}


}
