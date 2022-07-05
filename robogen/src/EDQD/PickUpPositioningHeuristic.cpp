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
					Heuristic(robot, scenario), env(scenario->getEnvironment()), active_(false), targetPoint(osg::Vec3d(-1000, -1000, -1000)),
					targetAreaPosition_(scenario->getEnvironment()->getGatheringZone()->getPosition()){

		setPriority(0);
	}

	PickUpPositioningHeuristic::~PickUpPositioningHeuristic(){}

	osg::Vec2d PickUpPositioningHeuristic::step(){
		if (robot_ -> isBoundToResource()){
			setActive(false);
			if (resource_ -> pushedByMaxRobots())
				return Heuristic::driveToTargetPosition(osg::Vec2d(targetAreaPosition_.x(), targetAreaPosition_.y()));
			else
				return osg::Vec2d(0.5, 0.5);
		}

		if (resource_ == NULL || !resource_->canBePickedup()){
			setActive(false);
			return osg::Vec2d(-1000, -1000);
		}
		if (resource_ -> pickup(robot_)){
			setActive(false);
			if(resource_ -> pushedByMaxRobots()){
				//std::cout << "Robot " << robot_ -> getId() << "[P-U-P] -- number of pushing robots allow resource to be moved: " << resource_ -> getNumberPushingRobots() << std::endl;
				return Heuristic::driveToTargetPosition(osg::Vec2d(targetAreaPosition_.x(), targetAreaPosition_.y()));
			}
			else{
				//std::cout << "Robot " << robot_ -> getId() << "[P-U-P] -- waiting for help, number of pushing robots: " << resource_ -> getNumberPushingRobots() << std::endl;
				return osg::Vec2d(0.5, 0.5);
			}
		}

		osg::Vec3d newPosition = nextStep(resource_);
		//std::cout << "Robot " << robot_ -> getId() << " stepping p-u-p [NEXT-STEP DONE]" << std::endl;
		if (resource_ != NULL && ( newPosition.x() != -1000 || newPosition.y() != -1000)  ){
			//std::cout << "Current position: (" << robot_->getCoreComponent()->getRootPosition().x() << ", " << robot_->getCoreComponent()->getRootPosition().y() << "). Driving to: (" << newPosition.x() << ", " << newPosition.y() << std::endl;
			return driveToTargetPosition(osg::Vec2d(newPosition.x(), newPosition.y()));
		}
		setActive(false);
		return osg::Vec2d(-1000, -1000);
	}

	osg::Vec3d PickUpPositioningHeuristic::nextStep(boost::shared_ptr<BoxResource> resource){
		osg::Vec3d robotPosition = robot_->getCoreComponent()->getRootPosition();

		Face stickyFace = resource->getStickyFace();
		Face robotFace = resource->getFaceClosestToPoint(robotPosition);

		// same facee or face not yet set
		if (stickyFace == robotFace || stickyFace == Face::NONE) {
			// Head for the anchor point
			return calculateTargetPoint(resource);
		}
		 else { // Different side, navigate to corner
			osg::Vec3d corner;
			// to "pad" the corner by radius x radius
			float radius = 0.2;
			float halflen_x = (float) resource->getGeomSize().x() / 2;
			float halflen_y = (float) resource->getGeomSize().y() / 2;
			if (robotFace == Face::LEFT) {
				if (stickyFace == Face::BACK) {
					// Back left corner
					corner = osg::Vec3d(-halflen_x - radius, halflen_y + radius, 0);
					//std::cout << "Closest face to robot ID: " << robot_ -> getId() << " is the left. sticky face is at the back moving to back left corner" << std::endl;
				} else {
					// Front left corner
					corner = osg::Vec3d(-halflen_x - radius, -halflen_y - radius, 0);
					//std::cout << "Closest face to robot ID: " << robot_ -> getId() << " is the left. sticky face is at the front or right moving to front left corner" << std::endl;
				}
			} else if (robotFace == Face::RIGHT) {
				if (stickyFace == Face::BACK) {
					// Back right
					corner = osg::Vec3d(halflen_x + radius, halflen_y + radius, 0);
					//std::cout << "Closest face to robot ID: " << robot_ -> getId() << " is the right. sticky face is at the back moving to back right corner" << std::endl;
				} else {
					// Front right
					corner = osg::Vec3d(halflen_x + radius, -halflen_y - radius, 0);
					//std::cout << "Closest face to robot ID: " << robot_ -> getId() << " is the right. sticky face is at the front or left moving to front right corner" << std::endl;
				}
			} else if (robotFace == Face::BACK) {
				if (stickyFace == Face::LEFT) {
					// Back left
					corner = osg::Vec3d(-halflen_x - radius, halflen_y + radius, 0);
					//std::cout << "Closest face to robot ID: " << robot_ -> getId() << " is the back. sticky face is on the left moving to back left corner" << std::endl;
				} else {
					// Front right
					corner = osg::Vec3d(halflen_x + radius, halflen_y + radius, 0);
					//std::cout << "Closest face to robot ID: " << robot_ -> getId() << " is the back. sticky face is on the left moving to back left corner" << std::endl;
				}
			} else if (robotFace == Face::FRONT) {
				if (stickyFace == Face::LEFT) {
					// Front left
					corner = osg::Vec3d(-halflen_x - radius, -halflen_y - radius, 0);
					//std::cout << "Closest face to robot ID: " << robot_ -> getId() << " is the front. sticky face is on the left moving to front left corner" << std::endl;
				} else {
					// Front right
					corner = osg::Vec3d(halflen_x + radius, -halflen_y - radius, 0);
					//std::cout << "Closest face to robot ID: " << robot_ -> getId() << " is the front. sticky face is on the left moving to front right corner" << std::endl;
				}
			}
			//corner.set(corner.x() + radius, corner.y() + radius, 0);
			//corner.addLocal(Math.copySign(radius, corner.x), Math.copySign(radius, corner.y));

			// Transform relative to resource
			//std::cout << "Local coordinates: (" << corner.x() << ", " << corner.y() << ")" << std::endl;
			//std::cout << "Global coordinates: (" << resource -> getLocalPointToOut(corner).x() << ", " << resource -> getLocalPointToOut(corner).y() << ")" << std::endl;

			return resource -> getLocalPointToOut(corner);

		}
	}

	osg::Vec3d PickUpPositioningHeuristic::calculateTargetPoint(boost::shared_ptr<BoxResource> resource){
		// Remember where we're headed, don't get stuck choosing sides
		/**if (targetPoint.x() != -1000) {
			return targetPoint;
		}*/
		/**if (target_ != NULL && !target_->isTaken()) {
			return target_ -> getPosition();
		}*/
		osg::Vec3d robotPosition = robot_ -> getCoreComponent() -> getRootPosition();
		boost::shared_ptr<AnchorPoint> anchorPoint = resource -> getClosestAnchorPoint(robotPosition);
		// Get the anchor point
		if (anchorPoint == NULL || anchorPoint -> isTaken() ){
			targetPoint =  osg::Vec3d(-1000, -1000, -1000);
			return targetPoint;
		}

		targetPoint = anchorPoint -> getPosition();
		//std::cout << "anchor point 1 - taken: " <<  resource_ -> getAnchorPoints()[0] ->isTaken() << " local pos: (" << resource_ -> getAnchorPoints()[0] -> getLocalPosition().x() << ", " << resource_ -> getAnchorPoints()[0] -> getLocalPosition().y() << ")\n";
		//std::cout << "anchor point 2 - taken: " <<  resource_ -> getAnchorPoints()[1] ->isTaken() << " local pos: (" << resource_ -> getAnchorPoints()[1] -> getLocalPosition().x() << ", " << resource_ -> getAnchorPoints()[1] -> getLocalPosition().y() << ")\n";
		//std::cout << "anchor point 3 - taken: " <<  resource_ -> getAnchorPoints()[0] ->isTaken() << " local pos: (" << resource_ -> getAnchorPoints()[2] -> getLocalPosition().x() << ", " << resource_ -> getAnchorPoints()[2] -> getLocalPosition().y() << ")\n";
		//std::cout << "robot ID: " << robot_ -> getId() << " is heading to anchor point position: (" << anchorPoint ->getLocalPosition().x() << "/"<<anchorPoint ->getPosition().x()<<", " << anchorPoint ->getLocalPosition().y() << "/" << anchorPoint ->getPosition().y() << ")" << std::endl;
		return targetPoint;
	}

	void PickUpPositioningHeuristic::setResource(boost::shared_ptr<BoxResource> resource){
		resource_ = resource;
	}
}
