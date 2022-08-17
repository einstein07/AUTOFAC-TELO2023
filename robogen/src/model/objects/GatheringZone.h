/*
 * @(#) GatheringZone.h   1.0   July 10, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 */
#ifndef ROBOGEN_GATHERING_ZONE_H_
#define ROBOGEN_GATHERING_ZONE_H_

#include <osg/Quat>
#include <osg/Vec3>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/shared_ptr.hpp>
#include "model/objects/Obstacle.h"
#include "BoxResource.h"
#include "Robogen.h"
#include "EDQD/Util.h"



namespace robogen {

/**
 * A simple gathering zone made of a box of specified size
 */
class GatheringZone : public Obstacle {

    public:

        /**
         * Initializes a gathering zone
         */
        GatheringZone(
                    dWorldID odeWorld, 
                    dSpaceID odeSpace, 
                    const osg::Vec3& pos,
                    const osg::Vec3& size
                );

        /**
         * Remove from world
         */
        virtual void remove();

        /**
         * Destructor
         */
        virtual ~GatheringZone();

        /**
         * Inherited from PositionObservable
         */
        virtual const osg::Vec3 getPosition();
        virtual const osg::Quat getAttitude();

        /**
         * @return the box size
         */
        const osg::Vec3 getSize();

        /**
         * @return the box size
         */
        void getAABB(
                    double& minX, 
                    double& maxX, 
                    double& minY,
                    double& maxY, 
                    double& minZ, 
                    double& maxZ
                );
        
        void step(std::vector<boost::shared_ptr<Robot> > robots, std::vector<boost::shared_ptr<BoxResource>> resources, boost::mutex& queueMutex);


        /**
         * @return the number of resources gathered thus far
         */
        int getNumberOfContainedResources();
        
        /**
         * @param BoxResource the resource that is being added to the gathering
         * zone
         */
        void addResource(std::vector<boost::shared_ptr<Robot> > robots, boost::shared_ptr<BoxResource> resource);
        /**
		 * @param BoxResource the resource that is being removed from the gathering
		 * zone
		 */
        void removeResource(std::vector<boost::shared_ptr<Robot> > robots, boost::shared_ptr<BoxResource> resource);
		/**
		 * Finds robots close to the resource that can be blamed for pushing the resource
		 * in/out of the gathering zone
		 */
		std::set<boost::shared_ptr<Robot> > findRobotsNearResource(std::vector<boost::shared_ptr<Robot> > robots, boost::shared_ptr<BoxResource>);


	private:
		/**
		 * Area space to ensure this object can be detected by sensors
		 */
        //dSpaceID areaSpace_;

        /**
         * The box
         */
        dBodyID box_;

        /**
         * The box
         */
        dGeomID boxGeom_;

        /**
         * The box size
         */
        osg::Vec3 size_;
        
        /**
         * A set of the resources gathered thus far
         */
        std::set<boost::shared_ptr<BoxResource> > containedResources_;

        /**
		 * Object information used by sensor to identify the nature of this object
		 */
		ObjectData data_;
		double BLAME_BOX_EXPANSION_RATE;
		int BLAME_BOX_TRIES;
    };

}

#endif /* ROBOGEN_GATHERING_ZONE_H_ */
