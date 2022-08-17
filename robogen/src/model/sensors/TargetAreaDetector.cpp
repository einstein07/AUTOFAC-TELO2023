/*
 * @(#) TargetAreaDetector.cpp   1.0   Oct 16, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#include "TargetAreaDetector.h"

#include <algorithm>
#include <iostream>
#include <cmath>

namespace robogen {

//const float TargetAreaDetector::DETECTOR_RANGE = /**0.255*/1;
//dGeomID TargetAreaDetector::RAY_GEOM = 0;

TargetAreaDetector::TargetAreaDetector(dSpaceID odeSpace,
		std::vector<boost::shared_ptr<SimpleBody> > sensorBodies,
		std::string baseLabel): odeSpace_(odeSpace), lastReadOutput_(0){
	for(unsigned int i=0; i<sensorBodies.size(); i++){
		for (dGeomID g=dBodyGetFirstGeom(sensorBodies[i]->getBody()); g
				; g=dBodyGetNextGeom(g)){
			sensorGeoms_.push_back(g);
		}
	}
	raySpace_ = dHashSpaceCreate(0);
	// set space sublevel
	dSpaceSetSublevel (raySpace_, 2);

	//Distance to detected object
	/**sensors_.push_back(
			boost::shared_ptr<Sensor>(new IrSensorElement(baseLabel,
					IrSensorElement::IR)));*/

	//Sense if the detected object is the target area
		sensors_.push_back(
			boost::shared_ptr<Sensor>(new TargetAreaDetectorElement(baseLabel)));
}

TargetAreaDetector::~TargetAreaDetector() {
	dSpaceDestroy(raySpace_);
}



struct RayTrace {
	std::vector<dGeomID> ignoreGeoms;
	bool isColliding;
	osg::Vec3 collisionPoint;
	ObjectData *objData;
};


void TargetAreaDetector::collisionCallback(void *data, dGeomID o1, dGeomID o2) {

	RayTrace *r = (RayTrace*) data;


	if (dGeomIsSpace (o1) || dGeomIsSpace (o2)) {
		// colliding a space with something :
		dSpaceCollide2 (o1, o2, data, collisionCallback);
	}
	else{
		// ignore what needs ignoring
		if (std::find(r->ignoreGeoms.begin(),r->ignoreGeoms.end(),
				o1) != r->ignoreGeoms.end() ||
				std::find(r->ignoreGeoms.begin(),r->ignoreGeoms.end(),
						o2) != r->ignoreGeoms.end()){
			return;
		}

		// if colliding get collision point
		dContact contact;

		if (dCollide(o1,o2,1,&contact.geom,sizeof(contact))) {
			r->collisionPoint =  osg::Vec3(contact.geom.pos[0],
					contact.geom.pos[1],
					contact.geom.pos[2]);
			r->isColliding = true;
			r->objData = (ObjectData*)dGeomGetData(o1);
			if(r->objData == NULL){
				r->objData = (ObjectData*)dGeomGetData(o2);
			}
		}
	}
}

void TargetAreaDetector::update(const osg::Vec3& position, const osg::Quat& attitude, boost::shared_ptr<Environment>& env) {
	this->position_ = position;
	this->attitude_ = attitude;

	osg::Vec3 rayVector = attitude_ *  osg::Vec3(1,0,0);

	// create ray
	dGeomID ray = dCreateRay(raySpace_, /**DETECTOR_RANGE*/1);
	// position ray
	dGeomRaySet(ray, position_.x(), position_.y(), position_.z(),
			rayVector.x(), rayVector.y(), rayVector.z());

	// prepare collision data structure
	RayTrace data;

	// ignore sensor bodies
	data.ignoreGeoms.insert(data.ignoreGeoms.end(),
					sensorGeoms_.begin(), sensorGeoms_.end());

	data.isColliding = false;
	data.objData = 0;
	// perform collision
	dSpaceCollide2((dGeomID)raySpace_, (dGeomID)odeSpace_, (void*)&data,
			TargetAreaDetector::collisionCallback);

	float value = 0.0;

	//std::cout << "Target area-UPATE. isColliding: " << data.isColliding << std::endl;

	//float distance = /**SENSOR_RANGE*/ 0.0;
	// ray should be capped at SENSOR_RANGE, but just to make sure we don't
	// allow bigger values here
	/**if(data.isColliding &&
			(data.collisionPoint - position_).length() < DETECTOR_RANGE) {
		distance = (data.collisionPoint - position_).length();

		//std::cout << "pos: " << position_.x() << " " << position_.y() <<  " " << position_.z() << std::endl;
		//std::cout << "ray: " << rayVector.x() << " " << rayVector.y() <<  " " << rayVector.z() << std::endl;
		std::cout << "Distance to object: " << distance << std::endl;

	}*/

	if (data.isColliding && data.objData != 0 ){
		if ( data.objData->isTargetArea ){
			value = 1.0;
		}
	}

	// want 0 when nothing is seen
	sensors_[0]->updateValue(value);


	dGeomDestroy(ray);



}


}







