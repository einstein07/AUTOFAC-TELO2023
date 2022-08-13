/*
 * @(#) BoxObstacle.cpp   1.0   July 14, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * @(#) $Id$
 */
#include "model/objects/BoxResource.h"
#include "utils/RobogenUtils.h"
#include "EDQD/Util.h"

namespace robogen {

BoxResource::BoxResource(dWorldID odeWorld, dSpaceID odeSpace,
		const osg::Vec3& pos, const osg::Vec3& size, float density, 
                    int type, int& resourceId) : odeWorld_(odeWorld),
                    odeSpace_(odeSpace), size_(size), density_(density), isCollected_(false),
                    type_(type), stickyFace_(Face::NONE) {


    // since resource is not fixed, create body
    box_ = dBodyCreate(odeWorld);
    box_2 = box_;
    dMassSetBox(&massOde, density, size.x(), size.y(), size.z());
    dBodySetMass(box_, &massOde);
    boxGeom_ = dCreateBox(odeSpace, size.x(), size.y(), size.z());
    dGeomSetBody(boxGeom_, box_);
    dGeomSetPosition(boxGeom_, pos.x(), pos.y(), pos.z());
    // for some reason body/geom position do not get tied together as they
    // should, so we set the body position as well, and use it when getting
    // the position on non-stationary bodies
    dBodySetPosition(box_, pos.x(), pos.y(), pos.z());
    
    jointGroup_ = dJointGroupCreate(0);
    value_ = type;
    pushingRobots_ = type;
    id = resourceId;
    data_.objectId = resourceId;
    resourceId++;
    data_.isResource = true;
    data_.isRobot = false;
    data_.isTargetArea = false;
    dGeomSetData (boxGeom_, (void*)&data_);

    leftAnchorPoints_.reserve(pushingRobots_);
    rightAnchorPoints_.reserve(pushingRobots_);
    frontAnchorPoints_.reserve(pushingRobots_);
    backAnchorPoints_.reserve(pushingRobots_);
    initAnchorPoints();

}
void BoxResource::setFixed(){
	//dMassSetBox(&massOde, 0, getGeomSize().x(), getGeomSize().y(), getGeomSize().z());
	if (box_ != 0){
		box_ = 0;
		dGeomSetBody(boxGeom_, box_);
	}
	//dBodySetMass(box_, &massOde);
}
void BoxResource::setMovable(){
	if (box_ == 0){
		box_ = box_2;
		dGeomSetBody(boxGeom_, box_);
	}
}

BoxResource::~BoxResource() {
    if (jointGroup_) {
        dJointGroupDestroy(jointGroup_);
    }
}

//TODO: Determine z anchor point --- possibly use center of mass of robot root component
//TODO: Consider having one anchor points container. Initialize this container
//once the sticky face has been determined
void BoxResource::initAnchorPoints(){
    float halfWidth = (float) size_.x() / 2;
    float halfLength = (float) size_.y() / 2;
    float height = 0.0f;
    float widthSpacing = (float) size_.x() / pushingRobots_;
    float lengthSpacing = (float) size_.y() / pushingRobots_;
    
    for (unsigned int face = 1; face <= 4; face++){
    	for (unsigned int i = 0; i < pushingRobots_; ++i){
    	        float x, y, z;
    	        if (face == Face::LEFT) {
    				x = -halfWidth;
    				y = -halfLength + lengthSpacing * (i + 0.5f);
    				leftAnchorPoints_.push_back( boost::shared_ptr<AnchorPoint>(new AnchorPoint
										   (
											   box_,
											   osg::Vec3d(x, y, height),
											   Face::LEFT
										   )
									   )
    						);
    			} else if (face == Face::RIGHT) {
    				x = halfWidth;
    				y = -halfLength + lengthSpacing * (i + 0.5f);
    				rightAnchorPoints_.push_back( boost::shared_ptr<AnchorPoint>(new AnchorPoint
										   (
											   box_,
											   osg::Vec3d(x, y, height),
											   Face::RIGHT
										   )
					   	   	   	   	   )
    						);
    			} else if (face == Face::BACK) {
    				x = -halfWidth + widthSpacing * (i + 0.5f);
    				y = halfLength;
    				backAnchorPoints_.push_back( boost::shared_ptr<AnchorPoint>(new AnchorPoint
										   (
											   box_,
											   osg::Vec3d(x, y, height),
											   Face::BACK
										   )
									   )
    						);
    			} else if (face == Face::FRONT) {
    				x = -halfWidth + widthSpacing * (i + 0.5f);
    				y = -halfLength;
    				frontAnchorPoints_.push_back( boost::shared_ptr<AnchorPoint>(new AnchorPoint
										   (
											   box_,
											   osg::Vec3d(x, y, height),
											   Face::FRONT
										   )
									   )
    						);
    	        }
    	    }
    }
}
std::vector< boost::shared_ptr<AnchorPoint> > BoxResource::getAnchorPointsForFace(Face face){
	switch(face){
		case LEFT:
			return leftAnchorPoints_;
		case RIGHT:
			return rightAnchorPoints_;
		case FRONT:
			return frontAnchorPoints_;
		case BACK:
			return backAnchorPoints_;
		default:{
			std::vector< boost::shared_ptr<AnchorPoint> > empty;
			return empty;}
	}
}
void BoxResource::remove() {
	dGeomDestroy(boxGeom_);
	if (box_ != 0) {
        for(int i=0; i< dBodyGetNumJoints(box_); i++) {
            dJointDestroy(dBodyGetJoint(box_, i));
        }
        dBodyDestroy(box_);
	}
}

const osg::Vec3 BoxResource::getPosition() {
	if (box_!= 0) {
		const dReal* pos = dBodyGetPosition(box_);
		return osg::Vec3(pos[0], pos[1], pos[2]);
	}
	const dReal* pos = dGeomGetPosition(boxGeom_);
	return osg::Vec3(pos[0], pos[1], pos[2]);
}

const osg::Quat BoxResource::getAttitude() {
    dQuaternion boxQuat;
    dGeomGetQuaternion(boxGeom_, boxQuat);
    return osg::Quat(boxQuat[1], boxQuat[2], boxQuat[3], boxQuat[0]);

}

Face BoxResource::getStickyFace(){
    return stickyFace_;
}

const osg::Vec3 BoxResource::getGeomSize() {
    return size_;
}

void BoxResource::getAABB(double& minX, double& maxX, double& minY,
		double& maxY, double& minZ, double& maxZ) {

	dReal aabb[6];
	dGeomGetAABB(boxGeom_, aabb);
	minX = aabb[0];
	maxX = aabb[1];
	minY = aabb[2];
	maxY = aabb[3];
	minZ = aabb[4];
	maxZ = aabb[5];
}

int BoxResource::getSize() {
    return pushingRobots_;
}
dMass BoxResource::getMass(){
	return massOde;
}
bool BoxResource::isCollected(){
    return isCollected_;
}
double BoxResource::getValue(){
    return value_;
}
void BoxResource::setCollected(bool isCollected){
    /**if (isCollected_ == isCollected) {
        return;
    }*/
    dropOff();

    isCollected_ = isCollected;
}
void BoxResource::dropOff(){
	/*
	 * Sticky side could be unset if resource "bumped" into target area without
	 * robots creating joints with it
	*/
	if (stickyFace_ != Face::NONE) {

		std::map<boost::shared_ptr<Robot>, dJointID>::iterator it = joints_.begin();
		for (; it != joints_.end(); ++it){
			it->first->setBoundToResource(false);
		}
		// Break all the joints
		dJointGroupEmpty(jointGroup_);
		joints_.clear();

		// Reset the anchor points
		std::vector<boost::shared_ptr<AnchorPoint>> anchorPoints = getAnchorPointsForFace(stickyFace_);
		for (boost::shared_ptr<AnchorPoint> anchorPoint: anchorPoints){
			anchorPoint -> markNotTaken();
		}
		// Reset the sticky side
		stickyFace_ = Face::NONE;
	}
}

int BoxResource::getNumberPushingRobots(){
    return joints_.size();  
}

int BoxResource::getType(){
    return type_;
}

std::set<boost::shared_ptr<Robot> > BoxResource::getPushingRobots(){
    std::set<boost::shared_ptr<Robot> > pushingRobots;
    std::map<boost::shared_ptr<Robot>, dJointID>::iterator it = joints_.begin();
    for (; it != joints_.end(); ++it){
        pushingRobots.insert(it->first);
    }
    return pushingRobots;
}

bool BoxResource::pushedByMaxRobots(){
    return getNumberPushingRobots() >= pushingRobots_;
}

bool BoxResource::canBePickedup(){
    return !isCollected_ && !pushedByMaxRobots();
}
bool BoxResource::attachRobot(boost::shared_ptr<Robot> robot){
	if(getNumberPushingRobots() != 0){
		return false;
	}
	else if(joints_.count(robot) != 0){
		return false;
	}
	else{
		boost::shared_ptr<Model> robotBody = robot -> getBodyPart("Core");
		if (robotBody != NULL && (distance(robotBody->getRootPosition(), getPosition()) <= 0.1/**0.08*/) ){
			dJointID joint = dJointCreateFixed (odeWorld_, jointGroup_);
			dJointAttach (
							joint,
							robotBody->getRoot()->getBody(),
							box_
						);
			dJointSetFixed (joint);
			joints_.insert(std::pair<boost::shared_ptr<Robot>, dJointID>(robot, joint));
			robot -> setBoundToResource(true);
			return true;
		}
		else{
			return false;
		}
	}
}
bool BoxResource::pickup(boost::shared_ptr<Robot> robot){
    if(!canBePickedup()){
    	//std::cout << "Is collected or pushed by max robots - cannot be picked up, isCollected: "<< isCollected_ << " pushed-by-max-robots: "<< getNumberPushingRobots() << std::endl;
        return false;
    }
    if(joints_.count(robot) != 0){
    	//std::cout << "Robot appears in list of pushing robots already - cannot be picked up." << std::endl;
        return false;
    }

    osg::Vec3 robotPositionLocal = getLocalPoint(robot->getCoreComponent()->getRootPosition());
   
    /** Check the face that the robot is attempting to attach to,only permit
     * the robot to attach to the sticky face
     */
    /********************
     * Want to make sure the sticky face is re-determined everytime a robot comes into close proximity
     * to the resource, but this can only work for instances where there won't be cooperation needed
     */
    /*if (getNumberPushingRobots() == 0){
    	stickyFace_ = Face::NONE;
    }*/
    /************************
     *
     */
    Face attachFace = getFaceClosestToPointLocal(robotPositionLocal);
    if (stickyFace_ != Face::NONE && stickyFace_ != attachFace) {
    	//std::cout << "Sticky face not NONE and NOT equal to attach-face - cannot be picked up. ACTIVE FACE: " << stickyFace_ << std::endl;
        return false;
    }
    
    boost::shared_ptr<AnchorPoint> closestAnchorPoint = getClosestAnchorPointLocal(robotPositionLocal);
    if (closestAnchorPoint == NULL){
    	//std::cout << "Anchor point null" <<std::endl;
    	return false;
    }

    	osg::Vec3d anchorPoint = closestAnchorPoint->getPosition();
    	boost::shared_ptr<Model> robotBody = robot -> getBodyPart("Core");
    	boost::shared_ptr<Model> s29Body = robot -> getBodyPart("S29");
    	boost::shared_ptr<Model> s30Body = robot -> getBodyPart("S30");
    	osg::Vec3d point = (s29Body -> getRootPosition() + s30Body -> getRootPosition())/2;
		if (robotBody != NULL && pushingRobots_ == 1 && (distance(robotBody->getRootPosition(), anchorPoint) > 0.080/*0.1.085*/) ){
			return false;
		}
		if (robotBody != NULL && pushingRobots_ > 1 && (distance(/*robotBody->getRootPosition()*/point, anchorPoint) > 0.03/**0.05*/) ){
			//std::cout << "Distance to core: " << distance(robotBody->getRootPosition(), anchorPoint) /**<< " Anchor point: " << anchorPoint.x() << " " << anchorPoint.y()*/ << " distance to point " << distance(point, anchorPoint) << std::endl; //" Robot: " << robotBody -> getRootAttitude().x() << " " << robotBody -> getRootAttitude().y() << " " << robotBody -> getRootAttitude().z()  <<  std::endl;
			return false;
		}
		/*osg::Vec3d calcLocalPos = getLocalPoint(osg::Vec3d(anchorPoint.x(), anchorPoint.y(), 0));

		double y = calcLocalPos.y();// - coreGlobalPos.y();
		double x = calcLocalPos.x();// - coreGlobalPos.x();
		double targetAngle = atan2 (y, x);
		osg::Vec3d coreGlobalPos = robot->getCoreComponent()->getRoot()->getPosition();
		//osg::Vec3d coreGlobalPos2 = robot_->getCoreComponent()->getRootPosition();
		osg::Vec3d robbotCalcLocalPos = getLocalPoint(robot->getCoreComponent()->getRoot()->getPosition());
		if(std::abs(targetAngle) > M_PI){
			if(signbit(targetAngle))
				targetAngle = M_PI*(-1);
			else
				targetAngle = M_PI;
		}
		if (targetAngle > -10 && targetAngle < 10){*/
		/*if ( (abs( anchorPoint.x() - robot->getCoreComponent()->getSlotPosition(2).x() ) < deltaX)
				&& (abs( anchorPoint.y() - robot->getCoreComponent()->getSlotPosition(2).y() ) < deltaY) ){*/
		// Set the sticky side if unset
		if (stickyFace_ == NONE) {
			stickyFace_ = attachFace;
		}
		dJointID joint = dJointCreateFixed (odeWorld_, jointGroup_);
		dJointAttach (
						joint,
						robotBody->getRoot()->getBody(),
						box_
					);
		dJointSetFixed (joint);
		joints_.insert(std::pair<boost::shared_ptr<Robot>, dJointID>(robot, joint));
		robot -> setBoundToResource(true);
		robot -> setBoundResourceId( getId());
		closestAnchorPoint -> markTaken();
		return true;
		/**}
		else{
			return false;
		}*/


}

osg::Vec3 BoxResource::getLocalPoint(osg::Vec3 globalPoint){
    dVector3 localPoint;
    if (box_ != 0){
    	dBodyGetPosRelPoint (
                        box_, 
                        globalPoint.x(), 
                        globalPoint.y(), 
                        globalPoint.z(), 
                        localPoint
                    );
    }
    else{
    	dGeomGetPosRelPoint (
    	                        boxGeom_,
    	                        globalPoint.x(),
    	                        globalPoint.y(),
    	                        globalPoint.z(),
    	                        localPoint
    	                    );
    }
    return osg::Vec3(localPoint[0], localPoint[1], localPoint[2]);
}
osg::Vec3 BoxResource::getLocalPointToOut(osg::Vec3d localPoint){

	dVector3 result;
	if (box_ != 0){
		dBodyGetRelPointPos (
					box_,
					localPoint.x(),
					localPoint.y(),
					localPoint.z(),
					result
				);
	}
	else{
		dGeomGetRelPointPos (
					boxGeom_,
					localPoint.x(),
					localPoint.y(),
					localPoint.z(),
					result
				);
	}
	return osg::Vec3(result[0], result[1], result[2]);

}
Face BoxResource::getFaceClosestToPoint(osg::Vec3 point){
	return getFaceClosestToPointLocal(getLocalPoint(point));
}

Face BoxResource::getFaceClosestToPointLocal(osg::Vec3 localPoint){
    float halfWidth = (float) size_.x() / 2;
    float halfLength = (float) size_.y() / 2;
    Face face;
    if (localPoint.y() > -halfLength && localPoint.y() < halfLength){
        if (localPoint.x() > 0){
            face = Face::RIGHT;
        }else{
            face = Face::LEFT;
        }
    }else if (localPoint.x() > -halfWidth && localPoint.x() < halfWidth){
        if(localPoint.y() > 0){
            face = Face::BACK;
        }else{
            face = Face::FRONT;
        }
    }else if (std::abs(localPoint.x()) - halfWidth > std::abs(localPoint.y()) - halfLength){
        if (localPoint.x() > 0) {
            face = Face::RIGHT;
        } else {
            face = Face::LEFT;
        }
    } else/* if (std::abs(localPoint.x()) - halfWidth < std::abs(localPoint.y()) - halfLength) */{
        if (localPoint.y() > 0) {
            face = Face::BACK;
        } else {
            face = Face::FRONT;
        }
    }
    return face;
}

boost::shared_ptr<AnchorPoint> BoxResource::getClosestAnchorPointLocal(osg::Vec3d localPoint) {
	// Get the side and corresponding anchor points
	Face face = (stickyFace_ != Face::NONE) ? stickyFace_ : getFaceClosestToPointLocal(localPoint);
	std::vector<boost::shared_ptr<AnchorPoint>> anchorPoints = getAnchorPointsForFace(face);
	// Fast path for single robot resource
	if (pushingRobots_ == 1) {
		boost::shared_ptr<AnchorPoint> anchorPoint = anchorPoints[0];
		return !anchorPoint->isTaken()? anchorPoint: NULL;
	}


	// Else iterate through anchor points finding closest one (generally only 2 options)
	boost::shared_ptr<AnchorPoint> closestAnchorPoint;
	float shortestDistance = std::numeric_limits<float>::max();
	for (boost::shared_ptr<AnchorPoint> anchor: anchorPoints) {
		if ( !anchor->isTaken()) {
			float distance_ = distance(anchor->getLocalPosition(), localPoint);/*sqrt(pow(anchorPoints_[i]->getPosition().x() - localPoint.x(), 2)
									+ pow(anchorPoints_[i]->getPosition().y() - localPoint.y(), 2)
									+ pow(anchorPoints_[i]->getPosition().z() - localPoint.z(), 2));*/
			if (distance_ < shortestDistance) {
				shortestDistance = distance_;
				closestAnchorPoint = anchor;

			}
		}
	}
	return closestAnchorPoint;
}

boost::shared_ptr<AnchorPoint> BoxResource::getClosestAnchorPoint(osg::Vec3 position){
    return getClosestAnchorPointLocal(getLocalPoint(position));
}

}/*Close namespace brace*/
