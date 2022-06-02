/*
 * @(#) BoxObstacle.cpp   1.0   July 14, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * @(#) $Id$
 */
#include "model/objects/BoxResource.h"
#include "utils/RobogenUtils.h"
namespace robogen {

BoxResource::BoxResource(dWorldID odeWorld, dSpaceID odeSpace,
		const osg::Vec3& pos, const osg::Vec3& size, float density, 
                    int type, int& resourceId) : odeWorld_(odeWorld),
                    odeSpace_(odeSpace), size_(size), isCollected_(false),
                    type_(type), stickyFace_(Face::NONE) {


    // since resource is not fixed, create body
    box_ = dBodyCreate(odeWorld);

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
    value_ = 100;
    pushingRobots_ = 1;
    id = resourceId;
    data_.objectId = resourceId;
    resourceId++;
    data_.isResource = true;
    data_.isRobot = false;
    data_.isTargetArea = false;
    dGeomSetData (boxGeom_, (void*)&data_);
}

BoxResource::~BoxResource() {
    if (jointGroup_) {
        dJointGroupDestroy(jointGroup_);
    }
}

//TODO: Determine z anchor point --- possibly use center of mass of robot root component
//TODO: Consider having one anchor points container. Initialize this container
//once the sticky face has been determined
bool BoxResource::initAnchorPoints(){
    float halfWidth = (float) size_.x() / 2;
    float halfLength = (float) size_.y() / 2;
    float height = 0.0f;
    float widthSpacing = (float) size_.x() / pushingRobots_;
    float lengthSpacing = (float) size_.y() / pushingRobots_;
    
    for (unsigned int i = 0; i < pushingRobots_; ++i){
        float x, y, z;
        /**if (stickyFace_ == Face::FRONT){
            x = halfWidth;
            y = -halfLength + lengthSpacing * (i + 0.5);
        }else if (stickyFace_ == Face::BACK){
            x = -halfWidth;
            y = -halfLength + lengthSpacing * (i + 0.5);
        }else if (stickyFace_ == Face::LEFT){
            x = -halfWidth + widthSpacing * (i + 0.5);
            y = -halfLength;
        }else if (stickyFace_ == Face::RIGHT){ //Right face
            x = -halfWidth + widthSpacing * (i + 0.5);
            y = halfLength;*/
        if (stickyFace_ == Face::LEFT) {
			x = -halfWidth;
			y = -halfLength + lengthSpacing * (i + 0.5f);
		} else if (stickyFace_ == Face::RIGHT) {
			x = halfWidth;
			y = -halfLength + lengthSpacing * (i + 0.5f);
		} else if (stickyFace_ == Face::BACK) {
			x = -halfWidth + widthSpacing * (i + 0.5f);
			y = halfLength;
		} else if (stickyFace_ == Face::FRONT) {
			x = -halfWidth + widthSpacing * (i + 0.5f);
			y = -halfLength;
        }else{
            return false;
        }
        z = height;
        anchorPoints_.push_back(
               boost::shared_ptr<AnchorPoint>(new AnchorPoint
                       (
                           box_, 
                           osg::Vec3(x, y, z),
                           stickyFace_
                       )
               )
           );
    }
    return true;
    
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
bool BoxResource::isCollected(){
    return isCollected_;
}
double BoxResource::getValue(){
    return value_;
}
void BoxResource::setCollected(bool isCollected){
    if (isCollected_ == isCollected) {
        return;
    }
    dropOff();

    isCollected_ = isCollected;
}
void BoxResource::dropOff(){
	/*
	 * Sticky side could be unset if resource "bumped" into target area without
	 * robots creating joints with it
	*/
	//if (stickyFace_ != Face::NONE) {

		std::map<boost::shared_ptr<Robot>, dJointID>::iterator it = joints_.begin();
		for (; it != joints_.end(); ++it){
			it->first->setBoundToResource(false);
		}
		// Break all the joints
		dJointGroupEmpty(jointGroup_);
		joints_.clear();

		// Reset the anchor points
		//anchorPoints_.clear();

		// Reset the sticky side
		//stickyFace_ = Face::NONE;
	//}
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
        return false;
    }
    if(joints_.count(robot) != 0){
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
    if (getNumberPushingRobots() == 0){
    	stickyFace_ = Face::NONE;
    }
    /************************
     *
     */
    Face attachFace = getFaceClosestToPointLocal(robotPositionLocal);
    if (stickyFace_ != Face::NONE && stickyFace_ != attachFace) {
        return false;
    }
    int closestAnchorIndex;
    
    if (!getClosestAnchorPointLocal(robotPositionLocal, closestAnchorIndex)) {
        return false; // Should not happen but apparently can...
    }

    boost::shared_ptr<AnchorPoint> anchor = anchorPoints_[closestAnchorIndex];
    if (anchor != NULL){
    	osg::Vec3d anchorPoint = anchor->getPosition();
		double deltaX = 0.08;
		double deltaY = 0.08;
		if ( (abs( anchorPoint.x() - robot->getCoreComponent()->getSlotPosition(2).x() ) < deltaX)
				&& (abs( anchorPoint.y() - robot->getCoreComponent()->getSlotPosition(2).y() ) < deltaY) ){

			if (createBallJoint(robot, anchor -> getPosition())){
				// Mark the anchor as taken and the robot as bound to a resource.
				anchor -> markTaken();
				robot -> setBoundToResource(true);
				dVector3 result;
				const dReal* pos = dBodyGetPosition(box_);
				dBodyGetPosRelPoint (
										robot -> getCoreComponent() -> getRoot() -> getBody(),
										pos[0],
										pos[1],
										pos[2],
										result
										);
				osg::Vec3d robotLocalPos = getLocalPoint( robot -> getCoreComponent() -> getRootPosition() );
				return true;
			}
		}
    }
    return false;
}

osg::Vec3 BoxResource::getLocalPoint(osg::Vec3 globalPoint){
    dVector3 localPoint;
    dBodyGetPosRelPoint (
                        box_, 
                        globalPoint.x(), 
                        globalPoint.y(), 
                        globalPoint.z(), 
                        localPoint
                    );
    return osg::Vec3(localPoint[0], localPoint[1], localPoint[2]);
}
osg::Vec3 BoxResource::getLocalPointToOut(osg::Vec3d localPoint){

	dVector3 result;
	dBodyGetRelPointPos (
					box_,
					localPoint.x(),
					localPoint.y(),
					localPoint.z(),
					result
				);

	return osg::Vec3(result[0], result[1], result[2]);

}
/*BoxResource::*/Face BoxResource::getFaceClosestToPoint(osg::Vec3 point){
	return getFaceClosestToPointLocal(getLocalPoint(point));
}

/**BoxResource::*/Face BoxResource::getFaceClosestToPointLocal(osg::Vec3 localPoint){
    float halfWidth = (float) size_.x() / 2;
    float halfLength = (float) size_.y() / 2;
    /**BoxResource::*/Face face;
    if (localPoint.y() > -halfLength && localPoint.y() < halfLength){
        if (localPoint.x() > 0){
            face = /**Face::FRONT*/ Face::RIGHT;
        }else{
            face = /**Face::BACK*/ Face::LEFT;
        }
    }else if (localPoint.x() > -halfWidth && localPoint.x() < halfWidth){
        if(localPoint.y() > 0){
            face = /**Face::RIGHT*/ Face::BACK;
        }else{
            face = /**Face::LEFT*/ Face::FRONT;
        }
    }else if (std::abs(localPoint.x()) - halfWidth > std::abs(localPoint.y()) - halfLength){
        if (localPoint.x() > 0) {
            face = /**Face::FRONT*/ Face::RIGHT;
        } else {
            face = /**Face::BACK*/ Face::LEFT;
        }
    } else {
        if (localPoint.y() > 0) {
            face = /**Face::RIGHT*/ Face::BACK;
        } else {
            face = /**Face::LEFT*/ Face::FRONT;
        }
    }
    return face;
}

bool BoxResource::getClosestAnchorPointLocal(osg::Vec3 localPoint, int& index) {
        // Get the side and corresponding anchor points
        if (stickyFace_ == Face::NONE){
            stickyFace_ = getFaceClosestToPointLocal(localPoint);
            if (!initAnchorPoints()){
                return false;
            }
        }
        // Fast path for single robot resource
        if (pushingRobots_ == 1) {
            index = 0;
            return !anchorPoints_[index]->isTaken();
        }

        // Else iterate through anchor points finding closest one (generally only 2 options)
        float shortestDistance = std::numeric_limits<float>::max();
        bool returnVal = false;
        for (int i = 0; i < anchorPoints_.size(); i++) {
            if (!anchorPoints_[i]->isTaken()) {

                float distance = sqrt(pow(anchorPoints_[i]->getPosition().x() - localPoint.x(), 2)
                                    + pow(anchorPoints_[i]->getPosition().y() - localPoint.y(), 2)
                                    + pow(anchorPoints_[i]->getPosition().z() - localPoint.z(), 2));
                if (distance < shortestDistance) {
                    shortestDistance = distance;
                    index = i;
                }
                returnVal = true;
            }
        }

        return returnVal;
    }

bool BoxResource::getClosestAnchorPoint(osg::Vec3 position, int& index){
    return getClosestAnchorPointLocal(getLocalPoint(position), index);
}
boost::shared_ptr<AnchorPoint> BoxResource::getClosestAnchorPoint(osg::Vec3 position){
	int index = -1000;
	if (getClosestAnchorPoint(position, index)){
		return anchorPoints_[index];
	}
	else{
		return NULL;
	}
}

bool BoxResource::createBallJoint(boost::shared_ptr<Robot> robot, osg::Vec3 anchorPoint){
	//std::cout << "Inside create ball joint" << std::endl;
	boost::shared_ptr<Model> robotBody = robot -> getBodyPart("Core");
	/**std::cout  	<< "The ID of the returned body: "
				<< robotBody->getId()
				<< std:: endl;*/
	//osg::Vec3d bodypos = robotBody->getRootPosition();

	// Check robot is not unreasonably far away
	/**if (distance(robotBody->getRootPosition(), anchorPoint) > 0.17) {

		std::cout << "Distance to object: "<< distance(robotBody->getRootPosition(), anchorPoint) << " Far to attach" << std::endl;
		return false;
	}
	else{
		std::cout 	<< "Can attach to resource, distance less than 0.17: "
					<< distance(robotBody->getRootPosition(), anchorPoint)
					<< std::endl;
	}*/


	/**dJointID joint = dJointCreateBall (odeWorld_, jointGroup_);*/
	dJointID joint = dJointCreateFixed (odeWorld_, jointGroup_);
    /**std::cout << "-*-*-*-*-*-*-*-*-*-*- attaching " <<
            /**robot->getCoreComponent()->getRoot()->getBody()*/
    		/**->getRoot()->getBody()
                << " " << box_ << std::endl;*/
    dJointAttach (
                    joint, 
                    /**robot->getCoreComponent()->getRoot()->getBody(),*/
					robotBody->getRoot()->getBody(),
                    box_
                );
    /**dJointSetBallAnchor(joint, anchorPoint[0], anchorPoint[1], anchorPoint[2]);*/
    dJointSetFixed (joint);
    joints_.insert(std::pair<boost::shared_ptr<Robot>, dJointID>(robot, joint));
    return true;
}

double BoxResource::distance (osg::Vec3d a, osg::Vec3d b){
	double distance = sqrt(pow(a.x() - b.x(), 2)
						+ pow(a.y() - b.y(), 2)
						+ pow(a.z() - b.z(), 2));
	return distance;
}


}/*Close namespace brace*/
