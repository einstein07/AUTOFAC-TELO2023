/*
 * @(#) GatheringZone.h   1.0   July 10, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 */
#include "model/objects/GatheringZone.h"
#include "utils/RobogenUtils.h"
#include "EDQD/EDQDRobot.h"

namespace robogen {

GatheringZone::GatheringZone(dWorldID odeWorld, dSpaceID odeSpace,
							const osg::Vec3& pos, const osg::Vec3& size,
							const osg::Vec3& rotationAxis, float rotationAngle) : size_(size) {
	/**areaSpace_ = dHashSpaceCreate(odeSpace);
	dSpaceSetSublevel (areaSpace_, 2);*/
    // make body 0
    box_ = 0;
    boxGeom_ = dCreateBox(odeSpace, size.x(), size.y(), size.z());
    dGeomSetBody(boxGeom_, box_);
    dGeomSetPosition(boxGeom_, pos.x(), pos.y(), pos.z());
    if (rotationAngle >= RobogenUtils::EPSILON_2){
		osg::Quat rotation;
		rotation.makeRotate(osg::DegreesToRadians(rotationAngle),rotationAxis);
		dQuaternion quatOde;
		quatOde[0] = rotation.w();
		quatOde[1] = rotation.x();
		quatOde[2] = rotation.y();
		quatOde[3] = rotation.z();
		dGeomSetQuaternion(boxGeom_, quatOde);

	}

    data_.objectId = 0;// Placeholder id - TODO: will we require a unique ide in future
	data_.isRobot = false;
	data_.isResource = false;
	data_.isTargetArea = true;
	data_.isWall = false;
	dGeomSetData (boxGeom_, (void*)&data_);
	BLAME_BOX_EXPANSION_RATE = 1.5;
	BLAME_BOX_TRIES = 5;
	/**std::cout 	<< "Gathering zone geom id: "
				<< boxGeom_
				<< std::endl;*/
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
void GatheringZone::step(std::vector<boost::shared_ptr<Robot> > robots, std::vector<boost::shared_ptr<BoxResource>> resources, boost::mutex& queueMutex){
	// Gathering zone aabb
	double minX, maxX, minY, maxY, minZ, maxZ;
	getAABB(minX, maxX, minY, maxY, minZ, maxZ);
	for (auto resource : resources){
		// resource aabb
		double oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ;
		resource->getAABB(oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ);
		if ( isOverlap(minX, maxX, minY, maxY, minZ, maxZ, oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ) && !containedResourcesIds_.count(resource->getId()) ){
			{
				//std::cout << "[ADD] - count for resource " << resource -> getId() << ": " <<  containedResourcesIds_.count(resource->getId()) << std::endl;
				boost::lock_guard<boost::mutex> lock(queueMutex);
				addResource(robots, resource);
			}
		}
		else if (/**containedResources_.count(resource)*/!isOverlap(minX, maxX, minY, maxY, minZ, maxZ, oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ) && containedResourcesIds_.count(resource -> getId())){ //Resource was added but is no longer within GZ
				{
					//std::cout << "[REMOVE] - count for resource " << resource -> getId() << ": " <<  containedResourcesIds_.count(resource->getId()) << std::endl;
					boost::lock_guard<boost::mutex> lock(queueMutex);
					removeResource(robots, resource);
				}
			}
	}
	//std::cout << "Number of contained resources: " << containedResources_.size() << std::endl;
}
int GatheringZone::getNumberOfContainedResources() {
    return containedResourcesIds_.size();
}

//TODO: Complete function logic
void GatheringZone::addResource(std::vector<boost::shared_ptr<Robot> > robots, boost::shared_ptr<BoxResource> resource){
	if ( resource -> getNumberPushingRobots() ){
		if (abs((distance(osg::Vec3d(resource -> getPosition().x(), resource -> getPosition().y(), 0), osg::Vec3d(resource ->getPosition().x(), getPosition().y(), 0))) < 0.5)){
			//std::cout << "Resource attached to robot and within req distance." << std::endl;
			//std::pair<std::set<boost::shared_ptr<BoxResource> >::iterator, bool> insertStatus;
			//insertStatus = containedResources_.insert(resource);
			std::pair<std::set<int>::iterator, bool> insertStatus;
			/**insertStatus = */containedResourcesIds_.insert(resource -> getId());
			//if (insertStatus.second) {
				std::set<boost::shared_ptr<Robot> > pushingRobots = resource -> getPushingRobots();
				if (pushingRobots.empty()){ // Robot might have just dropped off resource due to drop off heuristic
					//std::cout << "Pushing robots empty!" <<std::endl;
					pushingRobots = findRobotsNearResource(robots, resource);
				}
				if (!pushingRobots.empty()){
					for (auto robot : pushingRobots ){

							boost::dynamic_pointer_cast<EDQDRobot>(robot) -> updateFitness(resource -> getPosition(),resource -> getSize(), resource -> getValue());
							boost::dynamic_pointer_cast<EDQDRobot>(robot) -> incResourceCounter(resource -> getType());
							//std::cout << "GZ -inc resource counter for robot id: " << robot -> getId() << std::endl;

						//std::cout 	<< "Resource added to gathering zone. Time bound to resource: "
						//			<<boost::dynamic_pointer_cast<EDQDRobot>(robot)-> getTimeResourceBound() << std::endl;
							boost::dynamic_pointer_cast<EDQDRobot>(robot) -> resetTimeResourceBound();

					}
				}

			//}
				// Mark resource as collected (this breaks the joints)
				resource->setCollected(true);
		}
	}
	else if (!resource -> getNumberPushingRobots()){
		//std::pair<std::set<boost::shared_ptr<BoxResource> >::iterator, bool> insertStatus;
		//insertStatus = containedResources_.insert(resource);
		std::pair<std::set<int>::iterator, bool> insertStatus;
		insertStatus = containedResourcesIds_.insert(resource -> getId());
		if (insertStatus.second) {
			std::set<boost::shared_ptr<Robot> > pushingRobots = findRobotsNearResource(robots, resource);

			if (!pushingRobots.empty()){
				for (auto robot : pushingRobots ){
						boost::dynamic_pointer_cast<EDQDRobot>(robot) -> incResourceCounter(resource -> getType());
						boost::dynamic_pointer_cast<EDQDRobot>(robot) -> updateFitness(resource -> getSize(), resource -> getValue());
						boost::dynamic_pointer_cast<EDQDRobot>(robot) -> resetTimeResourceBound();
				}
			}

				// Mark resource as collected (this breaks the joints)
				resource->setCollected(true);

		}
	}
    
}

void GatheringZone::removeResource(std::vector<boost::shared_ptr<Robot> > robots, boost::shared_ptr<BoxResource> resource){
	if (/*containedResources_.erase(resource)*/ containedResourcesIds_.erase(resource -> getId())){
		resource -> setCollected(false);
		std::set<boost::shared_ptr<Robot> > pushingRobots = findRobotsNearResource(robots, resource);
		if (!pushingRobots.empty()){
			for (auto robot : pushingRobots) {
				boost::dynamic_pointer_cast<EDQDRobot>(robot) -> decResourceCounter(resource -> getType());
				boost::dynamic_pointer_cast<EDQDRobot>(robot) -> updateFitness(resource -> getSize(),  -1/10);
			}
		}
	}
}


std::set<boost::shared_ptr<Robot> > GatheringZone::findRobotsNearResource(std::vector<boost::shared_ptr<Robot> > robots, boost::shared_ptr<BoxResource> resource){
	std::set<boost::shared_ptr<Robot> > robots_;

	// Robot aabb
	double minX, maxX, minY, maxY, minZ, maxZ;
	// resource aabb
	double oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ;
	resource->getAABB(oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ);
	for (unsigned int i = 0; i < robots.size(); i++){
		robots[i]->getAABB(minX, maxX, minY, maxY, minZ, maxZ);

		if ( isOverlap(minX, maxX, minY, maxY, minZ, maxZ, oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ) ){
			robots_.insert(robots[i]);
		}
	}

	if (!robots_.empty()){
		return robots_;
	}

	for (unsigned int i = 0; i < BLAME_BOX_TRIES; i++) {
		oMinX = oMinX * BLAME_BOX_EXPANSION_RATE;
		oMaxX = oMaxX * BLAME_BOX_EXPANSION_RATE;
		oMinY = oMinY * BLAME_BOX_EXPANSION_RATE;
		oMaxY = oMaxY * BLAME_BOX_EXPANSION_RATE;
		oMinZ = oMinZ * BLAME_BOX_EXPANSION_RATE;
		oMaxZ = oMaxZ * BLAME_BOX_EXPANSION_RATE;
		for (unsigned int j = 0; j < robots.size(); j++){
			robots[j]->getAABB(minX, maxX, minY, maxY, minZ, maxZ);

			if ( isOverlap(minX, maxX, minY, maxY, minZ, maxZ, oMinX, oMaxX, oMinY, oMaxY, oMinZ, oMaxZ) ){
				robots_.insert(robots[j]);
			}
		}

		if (!robots.empty()) {
			break;
		}
	}
	return robots_;
}


}
