/*
 * @(#) GatheringZoneRender.h   1.0   August 02, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include "model/objects/GatheringZone.h"
#include "render/callback/PositionObservableCallback.h"
#include "render/objects/GatheringZoneRender.h"

namespace robogen {

GatheringZoneRender::GatheringZoneRender(boost::shared_ptr<GatheringZone> gatheringZone) {

	rootNode_ = new osg::PositionAttitudeTransform();

	osg::ref_ptr<osg::Box> box(
			new osg::Box(osg::Vec3(0, 0, 0), fromOde(gatheringZone->getSize().x()),
					fromOde(gatheringZone->getSize().y()), fromOde(gatheringZone->getSize().z())));

	osg::ref_ptr<osg::ShapeDrawable> boxDrawable(
			new osg::ShapeDrawable(box.get()));
	boxDrawable->setColor(osg::Vec4(0, 1, 1, 1));
	osg::ref_ptr<osg::Geode> geode(new osg::Geode());
	geode->addDrawable(boxDrawable.get());
	rootNode_->addChild(geode);
	rootNode_->setUpdateCallback(new PositionObservableCallback(gatheringZone));

}

GatheringZoneRender::~GatheringZoneRender() {

}

osg::ref_ptr<osg::PositionAttitudeTransform> GatheringZoneRender::getRootNode() {
	return rootNode_;
}

}
