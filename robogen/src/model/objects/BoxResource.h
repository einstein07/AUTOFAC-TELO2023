/*
 * @(#) BoxResource.h   1.0   July 14, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * @(#) $Id$
 */
#ifndef ROBOGEN_BOX_RESOURCE_H_
#define ROBOGEN_BOX_RESOURCE_H_

#include <osg/Quat>
#include <osg/Vec3>
#include <cmath>
#include "Resource.h"
#include "Robogen.h"
#include "Robot.h"
#include "model/objects/AnchorPoint.h"

namespace robogen {
//    class AnchorPoint;
/**
 * A simple resource object made of a box of specified size
 */
class BoxResource : public Resource {

    public:
        


		/**
		 * Initializes a resource object
		 */
		BoxResource(dWorldID odeWorld, dSpaceID odeSpace, const osg::Vec3& pos,
				const osg::Vec3& size, float density, int type, int& resourceId);

		/**
		 * Remove from world
		 */
		virtual void remove();

		/**
		 * Destructor
		 */
		virtual ~BoxResource();

		/**
		 * Inherited from PositionObservable
		 */
		virtual const osg::Vec3 getPosition();
		virtual const osg::Quat getAttitude();

		/**
		 * @return the box size
		 */
		void getAABB(double& minX, double& maxX, double& minY,
				double& maxY, double& minZ, double& maxZ);

		/**
		 * @return the geometric size of the resource
		 */
		const osg::Vec3 getGeomSize();

		/**
		 * @return the size of the resource (based on how many robots are
		 * required to push it)
		 */
		int getSize();

		dMass getMass();

		/**
		 * @return the face that robots can currently attach to.
		 */
		Face getStickyFace();

		/**
		 * @return the number of robots currently pushing/attached to this
		 * resource.
		 */
		int getNumberPushingRobots();

		/**
		 * @return the type of this
		 * resource.
		 */
		int getType();
		/**
		 * @return the robots currently pushing/attached to this
		 * resource.
		 */
		std::set<boost::shared_ptr<Robot> > getPushingRobots();
	
		/**
		 * Check whether this resource already has the max number of robots
		 * attached to it.
		 * @return true if max number of robots attached, false otherwise.
		 */
		bool pushedByMaxRobots();

		/**
		 * Check whether this resource has been collected
		 * @return true if the resource has been marked as collected, i.e. it has
		 * passed into the gathering zone
		 */
		bool isCollected();

		/**
		 * Mark this object as collected. i.e. mark it as being in the target area.
		 * @param isCollected a bool value which is set to true if the resource is
		 * completely within the gathering zone, thus breaking all existing joints
		 * with the pushing robots.
		 */
		void setCollected(bool isCollected);

		/**
		 * Break all existing joints with the pushing robots.
		 */
		void dropOff();

		/**
		 * @return the value of the resource
		 */
		double getValue();

		/**
		 * Check whether this resource can be picked up
		 * @return true if the resource can be picked up, false otherwise
		 */
		bool canBePickedup();

		/**
		 * Try to attach this resource to the supplied robot. If successful, a
		 * fixed joint will be created between the resource and the root component
		 * of the robot.
		 * @return true if the resource pick up attempt was successful, false
		 * otherwise
		 */
		bool pickup(boost::shared_ptr<Robot> robot);

		bool attachRobot(boost::shared_ptr<Robot> robot);
		/**
		 * @return all the anchor points of this resource
		 */
		std::vector<boost::shared_ptr<AnchorPoint> > getAnchorPoints(){
			return anchorPoints_;
		}

		/**
		 * @param a position in world coordinates
		 * @param index the index of the closest available anchor point (in world
		 * coordinates)
		 * @return true is there is an anchor point nearby, false if none is
		 * available.
		 */
		bool getClosestAnchorPoint(osg::Vec3 position, int& index);

		boost::shared_ptr<AnchorPoint> getClosestAnchorPoint(osg::Vec3 position);

		/**
		 * @param point a position in global coordinates
		 * @return the face of the resource that is closest to the provided point.
		 */
		Face getFaceClosestToPoint(osg::Vec3 point);
		/**
		 * @param globalPoint point in global coordinates
		 * @return the supplied global point in local coordinates, i.e. relative
		 * to the resource
		 */
		osg::Vec3 getLocalPoint(osg::Vec3 globalPoint);

		/**
		 * @return the global position
		 */
		osg::Vec3 getLocalPointToOut(osg::Vec3d localPoint);

		//double distance (osg::Vec3d a, osg::Vec3d b);

		int getId(){return id;}

		void setFixed();
		void setMovable();
                
    private:
        
        /**
         * Initializes anchor points based on the resource's sticky face 
         * @return true if the anchor points on the given resource face are
         * initialized successfully, false otherwise
         */
        bool initAnchorPoints();
        
        /**
         * @param localPoint a position in local coordinates
         * @return the face of the resource that is closest to the provided point.
         */
        Face getFaceClosestToPointLocal(osg::Vec3 localPoint);
        
        
        /**
         * Get the closest anchor point to a position in world space
         * @param localPoint point in local coordinates
         * @param index the index of the available anchor point 
         * @return true if there is an anchor point nearby that has not been 
         * taken yet, or false if unavailable
         */
        bool getClosestAnchorPointLocal(osg::Vec3d localPoint, int& index);
        
        /**
         * @param robot the robot that is attaching to this resource
         * @param anchorPoint the anchor point of the ball joint that is to be created
         */
        bool createBallJoint(boost::shared_ptr<Robot> robot, osg::Vec3 anchorPoint);
        
        /**
         * ODE physics world
         */
        dWorldID odeWorld_;

        /**
         * ODE collision space
         */
        dSpaceID odeSpace_;
        
        dMass massOde;

        /**
         * The box
         */
        dBodyID box_;

        dBodyID box_2;

        /**
         * The box
         */
        dGeomID boxGeom_;
        
        dJointGroupID jointGroup_;
        
        /**
         * The face of the resource to which robots can attach themselves
         */
        Face stickyFace_;

        /**
         * The geometric size of the resource
         */
        osg::Vec3 size_;
        
        float density_;

        /**
         * Status of the resource
         */
        bool isCollected_;
        
        /**
         * The number of robots required to push the resource
         */
        int pushingRobots_;
        /**
		 * The type of the resource
		 */
		int type_;
        
        /**
         * The value of this resource 
         */
        double value_;
        
        /**
         * The points at which robots can attach to the resource
         */
        std::vector<boost::shared_ptr<AnchorPoint> > anchorPoints_;
        
        /**
         * Pending joints, and fixed joints
         */
        std::map<boost::shared_ptr<Robot>, dJointID> joints_; 

        /**
         * Object information used by sensor to identify the nature of this object
         */
        ObjectData data_;
        int id;
};

}


#endif /* ROBOGEN_BOX_RESOURCE_H_ */
