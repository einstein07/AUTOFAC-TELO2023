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
    class AnchorPoint{

        public:

            /**
             * Initializes an anchor point
             * @param position position local to the resource 
             */
            AnchorPoint(dBodyID bodyId, osg::Vec3 localPosition, 
                    BoxResource::Face face): box_(bodyId), localPosition_(localPosition),
                        face_(face), taken_(false){
            }
            AnchorPoint(){}
            /**
             * Mark this point as taken
             */
            void markTaken() {
                if (!taken_)
                    taken_ = true;
                else
                    std::cerr<<"Attachment point is already taken!"<<std::endl;

            }

            /**
             * Destructor
             */
            ~AnchorPoint(){}

            /**
             * @return the local position
             */
            const osg::Vec3 getLocalPosition() {
                return localPosition_;
            }

            /**
             * @return the global position
             */
            osg::Vec3 getPosition() {
                dVector3 result;
                dBodyGetRelPointPos (
                                box_, 
                                localPosition_.x(), 
                                localPosition_.y(), 
                                localPosition_.z(), 
                                result
                            );

                return osg::Vec3(result[0], result[1], result[2]);
            }

            /**
             * @return the box face
             */
            const BoxResource::Face getFace() {
                return face_;
            }

            /**
             * @return whether this point is taken - true - or not - false.
             */
            const bool isTaken() {
                return taken_;
            }

        private:

            /**
            * The box
            */
           dBodyID box_;
            /**
             * Local position
             */
            osg::Vec3 localPosition_;

            /**
             * Status of the anchor point
             */
            bool taken_;

            /**
             * Which face of the cube
             */
            BoxResource::Face face_;

};

BoxResource::BoxResource(dWorldID odeWorld, dSpaceID odeSpace,
		const osg::Vec3& pos, const osg::Vec3& size, float density, 
                    int pushingRobots) : size_(size), isCollected_(false), 
                        pushingRobots_(pushingRobots), stickyFace_(Face::NONE) {


    // since resource is not fixed, create body
    box_ = dBodyCreate(odeWorld);
    dMass massOde;
    dMassSetBox(&massOde, density, size.x(), size.y(), size.z());
    dBodySetMass(box_, &massOde);
    boxGeom_ = dCreateBox(odeSpace, size.x(), size.y(), size.z());
    dGeomSetBody(boxGeom_, box_);
    dGeomSetPosition(boxGeom_, pos.x(), pos.y(), pos.z());
    // for some reason body/geom position do not get tied together as they
    // should, so we set the body position as well, and use it when getting
    // the position on non-stationary bodies
    dBodySetPosition(box_, pos.x(), pos.y(), pos.z());
    
    jointGroup_ = dJointGroupCreate (0);
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
    float halfHeight = (float) size_.z() / 2;
    float widthSpacing = (float) size_.x() / pushingRobots_;
    float lengthSpacing = (float) size_.y() / pushingRobots_;
    
    for (unsigned int i = 0; i < pushingRobots_; ++i){
        float x, y, z;
        if (stickyFace_ == Face::FRONT){
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
            y = halfLength;
        }else{
            return false;
        }
        z = halfHeight;
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
        for(int i=0; i< dBodyGetNumJoints(box_); i++) {
            dJointDestroy(dBodyGetJoint(box_, i));
        }
        dBodyDestroy(box_);
}

const BoxResource::Face BoxResource::getStickyFace(){
    return stickyFace_;
}
const osg::Vec3 BoxResource::getPosition() {
    const dReal* pos = dBodyGetPosition(box_);
    return osg::Vec3(pos[0], pos[1], pos[2]);
}

const osg::Quat BoxResource::getAttitude() {
    dQuaternion boxQuat;
    dGeomGetQuaternion(boxGeom_, boxQuat);
    return osg::Quat(boxQuat[1], boxQuat[2], boxQuat[3], boxQuat[0]);

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

const int BoxResource::getSize() {
    return pushingRobots_;
}
const bool BoxResource::isCollected(){
    return isCollected_;
}
const double BoxResource::getValue(){
    return value_;
}
void BoxResource::setCollected(bool isCollected){
    if (isCollected_ == isCollected) {
        return;
    }
    /* 
     * Sticky side could be unset if resource "bumped" into target area without 
     * robots creating joints with it
    */
    if (isCollected && stickyFace_ != Face::NONE) {
        
        std::map<boost::shared_ptr<Robot>, dJointID>::iterator it = joints_.begin();
        for (; it != joints_.end(); ++it){
            it->first->setBoundToResource(false);
        }
        // Break all the joints
        dJointGroupEmpty(jointGroup_);
        joints_.clear();

        // Reset the anchor points
        anchorPoints_.clear();

        // Reset the sticky side
        stickyFace_ = Face::NONE;
    }

    isCollected_ = isCollected;
}
int BoxResource::getNumberPushingRobots(){
    return joints_.size();  
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
bool BoxResource::pickup(boost::shared_ptr<Robot> robot){
    if(!canBePickedup()){
        return false;
    }
    if(joints_.count(robot) != 0){
        return false;
    }
    osg::Vec3 robotPosition = robot->getCoreComponent()->getRootPosition();
    osg::Vec3 robotPositionLocal = getLocalPoint(robotPosition); 
   
    /** Check the face that the robot is attempting to attach to,only permit
     * the robot to attach to the sticky face
    **/
    Face attachFace = getFaceClosestToPointLocal(robotPositionLocal);
    if (stickyFace_ != Face::NONE && stickyFace_ != attachFace) {
        return false;
    }
    int closestAnchorIndex;
    
    if (!getClosestAnchorPointLocal(robotPositionLocal, closestAnchorIndex)) {
        return false; // Should not happen but apparently can...
    }
    // Check robot is not unreasonably far away
//    if (robotPositionLocal.sub(closestAnchor.getPosition()).length()
//            > robot.getRadius() * 2.5) {
//        return false;
//    }

    createBallJoint(robot, anchorPoints_[closestAnchorIndex]->getPosition());

    // Mark the anchor as taken and the robot as bound to a resource.
    anchorPoints_[closestAnchorIndex]->markTaken();
    robot->setBoundToResource(true);

    return true;
    
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

BoxResource::Face BoxResource::getFaceClosestToPointLocal(osg::Vec3 localPoint){
    float halfWidth = (float) size_.x() / 2;
    float halfLength = (float) size_.y() / 2;
    BoxResource::Face face;
    if (localPoint.y() > -halfLength && localPoint.y() < halfLength){
        if (localPoint.x() > 0){
            face = Face::FRONT;
        }else{
            face = Face::BACK;
        }
    }else if (localPoint.x() > -halfWidth && localPoint.x() < halfWidth){
        if(localPoint.y() > 0){
            face = Face::RIGHT;
        }else{
            face = Face::LEFT;
        }
    }else if (std::abs(localPoint.x()) - halfWidth > std::abs(localPoint.y()) - halfLength){
        if (localPoint.x() > 0) {
            face = Face::FRONT;
        } else {
            face = Face::BACK;
        }
    } else {
        if (localPoint.y() > 0) {
            face = Face::RIGHT;
        } else {
            face = Face::LEFT;
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

void BoxResource::createBallJoint(boost::shared_ptr<Robot> robot, osg::Vec3 anchorPoint){
    dJointID joint = dJointCreateBall (odeWorld_, jointGroup_);
    std::cout << "-*-*-*-*-*-*-*-*-*-*- attaching " << 
            robot->getCoreComponent()->getRoot()->getBody()
                << " " << box_ << std::endl;
    dJointAttach (
                    joint, 
                    robot->getCoreComponent()->getRoot()->getBody(), 
                    box_
                );
    dJointSetBallAnchor(joint, anchorPoint[0], anchorPoint[1], anchorPoint[2]);
    joints_.insert(std::pair<boost::shared_ptr<Robot>, dJointID>(robot, joint));
}

}/*Close namespace brace*/
