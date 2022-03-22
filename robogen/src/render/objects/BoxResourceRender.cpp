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

BoxResourceRender::BoxResourceRender(boost::shared_ptr<BoxResource> resource){

	/**this->rootNode_ = osg::ref_ptr<osg::PositionAttitudeTransform>(
				new osg::PositionAttitudeTransform());*/

	rootNode_ = new osg::PositionAttitudeTransform();

	osg::ref_ptr<osg::Box> box(
			new osg::Box(osg::Vec3(0, 0, 0), fromOde(resource->getGeomSize().x()),
					fromOde(resource->getGeomSize().y()), fromOde(resource->getGeomSize().z())));

	osg::ref_ptr<osg::ShapeDrawable> boxDrawable(
			new osg::ShapeDrawable(box.get()));
	boxDrawable->setColor(osg::Vec4(0, 1, 0, 1));
	osg::ref_ptr<osg::Geode> geode(new osg::Geode());
	geode->addDrawable(boxDrawable.get());
	rootNode_->addChild(geode);
	rootNode_->setUpdateCallback(new PositionObservableCallback(resource));


}

BoxResourceRender::~BoxResourceRender() {

}

osg::ref_ptr<osg::PositionAttitudeTransform> BoxResourceRender::getRootNode() {
	return rootNode_;
}

}
