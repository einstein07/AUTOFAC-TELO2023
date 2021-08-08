/*
 * @(#) GatheringZone.h   1.0   July 10, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 */
#include "model/objects/GatheringZone.h"
#include "utils/RobogenUtils.h"

namespace robogen {

GatheringZone::GatheringZone(dWorldID odeWorld, dSpaceID odeSpace,
		const osg::Vec3& pos, const osg::Vec3& size) : size_(size) {
    // make body 0
    box_ = 0;
    boxGeom_ = dCreateBox(odeSpace, size.x(), size.y(), size.z());
    dGeomSetBody(boxGeom_, box_);
    dGeomSetPosition(boxGeom_, pos.x(), pos.y(), pos.z());
}

GatheringZone::~GatheringZone() {
}

void GatheringZone::remove() {
    dGeomDestroy(boxGeom_);
}

const osg::Vec3 GatheringZone::getPosition() {
    const dReal* pos = dGeomGetPosition(boxGeom_);
    return osg::Vec3(pos[0], pos[1], pos[2]);
}

const osg::Quat GatheringZone::getAttitude() {
    dQuaternion boxQuat;
    dGeomGetQuaternion(boxGeom_, boxQuat);
    return osg::Quat(boxQuat[1], boxQuat[2], boxQuat[3], boxQuat[0]);
}

const osg::Vec3 GatheringZone::getSize() {
    return size_;
}

void GatheringZone::getAABB(double& minX, double& maxX, double& minY,
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

int GatheringZone::getNumberOfContainedResources() {
    return containedResources_.size();
}

//TODO: Complete function logic
bool GatheringZone::addResource(boost::shared_ptr<BoxResource> resource){
    std::pair<std::set<boost::shared_ptr<BoxResource> >::iterator, bool> insertStatus;
    insertStatus = containedResources_.insert(resource);
    if (insertStatus.second) {
        // Mark resource as collected (this breaks the joints)
        resource->setCollected(true);
        return true;
    }else{
        return false;
    }
    
}

}
