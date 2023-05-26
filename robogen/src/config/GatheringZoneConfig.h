/*
 * @(#) ObstaclesConfig.h   1.0   Mar 12, 2013
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * @(#) $Id$
 */
#ifndef ROBOGEN_GATHERINGZONE_CONFIG_H_
#define ROBOGEN_GATHERINGZONE_CONFIG_H_

#include <osg/Vec3>
#include <vector>
#include "robogen.pb.h"

namespace robogen {

/**
 * \brief Gathering zone configuration parameters
 */
class GatheringZoneConfig {

public:

	/**
	 * \brief Default Constructor
	 * Initializes gethering zone configuration
	 */
	GatheringZoneConfig() {}
	/**
	 * \brief Constructor
	 * Initializes gethering zone configuration
	 */
	GatheringZoneConfig(const osg::Vec3& position, const osg::Vec3& size) :
			position_(position), size_(size) {}

	/**
	 * Destructor
	 */
	virtual ~GatheringZoneConfig() {}

	/**
	 * Returns position of gathering zone.\n
	 * @return the coordinates of the gathering zone
	 * point of reference is its center
	 */
	const osg::Vec3& getPosition() const {
		return position_;
	}

	/**
	 * Returns size of gathering zone.\n
	 * @return the size of the gathering zone
	 */
	const osg::Vec3& getSize() const {
		return size_;
	}

	/**
	 * Serializes gathering zone into a SimulatorConf message
	 */
	void serialize(robogenMessage::SimulatorConf &message){
            message.set_gatheringzonex(position_.x());
            message.set_gatheringzoney(position_.y());
            message.set_gatheringzonez(position_.z());
            message.set_gatheringzonesizex(size_.x());
            message.set_gatheringzonesizey(size_.y());
            message.set_gatheringzonesizez(size_.z());
	}

private:

	/**
	 * Gathering zone coordinates
	 */
	osg::Vec3 position_;

	/**
	 * Gathering zone size
	 */
	osg::Vec3 size_;
};

}

#endif /* ROBOGEN_GATHERINGZONE_CONFIG_H_ */
