/*
 * @(#) ResourcesConfig.h   1.0   July 21, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#ifndef ROBOGEN_RESOURCES_CONFIG_H_
#define ROBOGEN_RESOURCES_CONFIG_H_

#include <osg/Vec3>
#include <vector>
#include "robogen.pb.h"

namespace robogen {

/**
 * Resources configuration parameters
 */
class ResourcesConfig {

public:

	/**
	 * Initializes resources configuration
	 */
	ResourcesConfig() {}

	ResourcesConfig(const std::vector<osg::Vec3>& coordinates,
			const std::vector<osg::Vec3>& sizes,
			const std::vector<float> &densities,
			const std::vector<int>& pushingRobots) :
			coordinates_(coordinates), sizes_(sizes), densities_(densities),
			pushingRobots_(pushingRobots) {

	}

	/**
	 * Destructor
	 */
	virtual ~ResourcesConfig() {

	}

	/**
	 * @return the coordinates of the resources
	 */
	const std::vector<osg::Vec3>& getCoordinates() const {
		return coordinates_;
	}

	/**
	 * @return the size of the resources
	 */
	const std::vector<osg::Vec3>& getSizes() const {
		return sizes_;
	}

	/**
	 * @return the densities of the resources
	 */
	const std::vector<float>& getDensities() const{
		return densities_;
	}

        /**
	 * @return the number of robots required to push the resources 
	 */
	const std::vector<int>& getNumberOfPushingRobots() const{
		return pushingRobots_;
	}
        
	/**
	 * Serialize resources into a SimulatorConf message
	 */
	void serialize(robogenMessage::SimulatorConf &message){
		for (unsigned int i=0; i<coordinates_.size(); ++i){
			robogenMessage::Resource *curr = message.add_resources();
			curr->set_density(densities_[i]);
                        curr->set_pushingrobots(pushingRobots_[i]);
			curr->set_x(coordinates_[i].x());
			curr->set_y(coordinates_[i].y());
			curr->set_z(coordinates_[i].z());
			curr->set_xsize(sizes_[i].x());
			curr->set_ysize(sizes_[i].y());
			curr->set_zsize(sizes_[i].z());
		}
	}

private:

	/**
	 * Resources coordinates
	 */
	std::vector<osg::Vec3> coordinates_;

	/**
	 * Resource sizes
	 */
	std::vector<osg::Vec3> sizes_;

	/**
	 * Resource densities
	 */
	std::vector<float> densities_;

	/**
	 * Number of robots required to push the resource
	 */
	std::vector<int> pushingRobots_;
};

}

#endif /* ROBOGEN_RESOURCES_CONFIG_H_ */
