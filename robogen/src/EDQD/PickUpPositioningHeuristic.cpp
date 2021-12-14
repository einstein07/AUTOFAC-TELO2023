/*
 * @(#) PickUpPositioningHeuristic.cpp   1.0   Oct 25, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#include "PickUpPositioningHeuristic.h"
#include "Heuristic.h"
#include "model/motors/RotationMotor.h"
#include "model/objects/AnchorPoint.h"

namespace robogen{

	PickUpPositioningHeuristic::PickUpPositioningHeuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario):
					Heuristic(robot, scenario), env(scenario->getEnvironment()),
					targetPoint(osg::Vec3d(-1000, -1000, -1000)), targetAreaPosition_(scenario->getEnvironment()->getGatheringZone()->getPosition()){

		setPriority(0);
	}

	PickUpPositioningHeuristic::~PickUpPositioningHeuristic(){}

	osg::Vec2d PickUpPositioningHeuristic::step(){
		if (resource_ -> pickup(robot_)){
			ACTIVE = false;

			robot_ -> setBoundResourceId(resource_ -> getId());
			std::cout 	<< "Robot - "
						<< robot_ -> getId()
						<< " picked up resource" << std::endl;
			if (resource_ -> getNumberPushingRobots() != resource_ -> getSize()){
				return osg::Vec2d(0, 0);
			}

			else{
				// Successfully attached to resource, now drive to target area to drop off resource
				resource_ = NULL;
				return Heuristic::driveToTargetPosition(osg::Vec2d(/**targetPos.x()*/targetAreaPosition_.x(), targetAreaPosition_.y()/**targetPos.y()*/));
			}
		}
		else{
			osg::Vec3d newPosition = nextStep(resource_);
			if (resource_ != NULL && ( newPosition.x() != -1000 || newPosition.y() != -1000)  ){
				std::cout 	<< "POSITIONING... "
							<< "Resource id: "
							<< resource_ -> getId()
							<< std::endl;
				return driveToTargetPosition(osg::Vec2d(newPosition.x(), newPosition.y()));
			}
			return osg::Vec2d(-1000, -1000);
		}
	}

	osg::Vec3d PickUpPositioningHeuristic::nextStep(boost::shared_ptr<BoxResource> resource){
		osg::Vec3d robotPosition = robot_->getCoreComponent()->getRootPosition();

		/*BoxResource::*/Face stickyFace = resource->getStickyFace();
		/**BoxResource::*/Face robotFace = resource->getFaceClosestToPoint(robotPosition);

		// same facee or face not yet set
		if (stickyFace == robotFace || stickyFace == /**BoxResource*/Face::NONE) {
			// Head for the anchor point
			//std::cout << "Heading to anchor point" <<std::endl;
			return calculateTargetPoint(resource);
		}
		 else { // Different side, navigate to corner
			 //std::cout << "Different side, navigate to corner." <<std::endl;
			osg::Vec3d corner;
			float halflen_x = (float) resource->getGeomSize().x() / 2;
			float halflen_y = (float) resource->getGeomSize().y() / 2;
			if (robotFace == /**BoxResource::*/Face::LEFT) {
				if (stickyFace == /**BoxResource::*/Face::BACK) {
					// Top left corner
					corner = osg::Vec3d(-halflen_x, halflen_y, 0);
				} else {
					// Bottom left corner
					corner = osg::Vec3d(-halflen_x, -halflen_y, 0);
				}
			} else if (robotFace == /**BoxResource::*/Face::RIGHT) {
				if (stickyFace == /**BoxResource::*/Face::BACK) {
					// Top right
					corner = osg::Vec3d(halflen_x, halflen_y, 0);
				} else {
					// Bottom right
					corner = osg::Vec3d(halflen_x, -halflen_y, 0);
				}
			} else if (robotFace == /**BoxResource::*/Face::BACK) {
				if (stickyFace == /**BoxResource::*/Face::LEFT) {
					// Top left
					corner = osg::Vec3d(-halflen_x, halflen_y, 0);
				} else {
					// Top right
					corner = osg::Vec3d(halflen_x, halflen_y, 0);
				}
			} else if (robotFace == /**BoxResource::*/Face::FRONT) {
				if (stickyFace == /**BoxResource::*/Face::LEFT) {
					// Bottom left
					corner = osg::Vec3d(-halflen_x, -halflen_y, 0);
				} else {
					// Bottom right
					corner = osg::Vec3d(halflen_x, -halflen_y, 0);
				}
			}

			// "Pad" the corner by radius x radius
			//float radius = robot.getRadius();
			//corner.addLocal(Math.copySign(radius, corner.x), Math.copySign(radius, corner.y));

			// Transform relative to resource

			return resource -> getLocalPointToOut(corner);

		}
	}

	osg::Vec3d PickUpPositioningHeuristic::calculateTargetPoint(boost::shared_ptr<BoxResource> resource){
		// Remember where we're headed, don't get stuck choosing sides
		if (targetPoint.x() != -1000 && targetPoint.y() != -1000) {
			return targetPoint;
		}
		osg::Vec3d robotPosition = robot_ -> getCoreComponent() -> getRootPosition();
		int index = -1000;
		// Get the anchor point and side normal
		if (resource -> getClosestAnchorPoint(robotPosition, index)){
			boost::shared_ptr<AnchorPoint> anchor = resource -> getClosestAnchorPoint(robotPosition);
			// Get a distance away from the anchor point
			targetPoint = anchor -> getPosition();
			/**std::cout 	<< "Anchor point: "
						<< "(" << targetPoint.x() << ", " << targetPoint.y() << ")"
						<<std::endl;*/
			return targetPoint;
		}
		else{
			return targetPoint;
		}
	}

	void PickUpPositioningHeuristic::setResource(boost::shared_ptr<BoxResource> resource){
		ACTIVE = true;
		resource_ = resource;
	}
}
