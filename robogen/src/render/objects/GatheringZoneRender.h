/*
 * @(#) GatheringZoneRender.h   1.0   August 02, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#ifndef ROBOGEN_GATHERING_ZONE_RENDER_MODEL_H_
#define ROBOGEN_GATHERING_ZONE_RENDER_MODEL_H_

#include <boost/shared_ptr.hpp>
#include <osg/PositionAttitudeTransform>

namespace robogen {

class GatheringZone;

class GatheringZoneRender {

public:

	GatheringZoneRender(boost::shared_ptr<GatheringZone> gatheringZone);

	virtual ~GatheringZoneRender();

	osg::ref_ptr<osg::PositionAttitudeTransform> getRootNode();

private:

	osg::ref_ptr<osg::PositionAttitudeTransform> rootNode_;


};


}


#endif /* ROBOGEN_GATHERING_ZONE_RENDER_MODEL_H_ */
