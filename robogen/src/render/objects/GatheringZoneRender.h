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
/**
 * \brief Class to render GatheringZone
 */
class GatheringZoneRender {

public:
	/**
	 * \brief Constructor
	 */
	GatheringZoneRender(boost::shared_ptr<GatheringZone> gatheringZone);
	/**
	 * \brief Destructor
	 */
	virtual ~GatheringZoneRender();
	/**
	 * \brief Returns root node
	 */
	osg::ref_ptr<osg::PositionAttitudeTransform> getRootNode();

private:

	osg::ref_ptr<osg::PositionAttitudeTransform> rootNode_;


};


}


#endif /* ROBOGEN_GATHERING_ZONE_RENDER_MODEL_H_ */
