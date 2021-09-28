/*
 * @(#) LowResCameraSensor.cpp   1.0   Aug 19, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#include "LowResCameraSensor.h"

#include <algorithm>
#include <iostream>
#include <cmath>

namespace robogen {

const float LowResCameraSensor::SENSOR_RANGE = 0.255;

LowResCameraSensor::LowResCameraSensor(dSpaceID odeSpace,
		std::vector<boost::shared_ptr<SimpleBody> > sensorBodies,
		std::string baseLabel):Sensor(baseLabel), odeSpace_(odeSpace), lastReadOutput_(0){
	objectId_ = -1;
	for(unsigned int i=0; i<sensorBodies.size(); i++){
		for (dGeomID g=dBodyGetFirstGeom(sensorBodies[i]->getBody()); g
				; g=dBodyGetNextGeom(g)){
			sensorGeoms_.push_back(g);
		}
	}
	raySpace_ = dHashSpaceCreate(0);


//	sensors_.push_back(
//		boost::shared_ptr<IrSensorElement>(new IrSensorElement(baseLabel,
//				IrSensorElement::IR)));
	// no ambient light sensing here for now
	/*sensors_.push_back(
			boost::shared_ptr<IrSensorElement>(new IrSensorElement(baseLabel,
				IrSensorElement::AMBIENT_LIGHT)));
	 */
}

LowResCameraSensor::~LowResCameraSensor() {
	dSpaceDestroy(raySpace_);
}



struct RayTrace {
	std::vector<dGeomID> ignoreGeoms;
	bool isColliding;
	osg::Vec3 collisionPoint;
	ObjectData *objData;
};

void LowResCameraSensor::collisionCallback(void *data, dGeomID o1, dGeomID o2){
	RayTrace *r = (RayTrace*) data;
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

		std::cout << "Getting pointer to geom data" << std::endl;
		r->objData = (ObjectData*)dGeomGetData(o1);
		if(r->objData == NULL){
			std::cout << "**Geom o1** is not either one of the objects of interest." << std::endl;
			r->objData = (ObjectData*)dGeomGetData(o2);
		}
		if(r->objData != NULL){
			std::cout << "**Geom o2** is an object of interest. Managed to get a valid pointer." <<std::endl;
			//Get the detected resource id. Use it in the model class to get information about the resource from the env shared pointer
			if (r->objData->isResource){
				//updateObjectId(r->objData->objectId);
			}
		}
	}
}

void LowResCameraSensor::update(const osg::Vec3& position, const osg::Quat& attitude) {
	this->position_ = position;
	this->attitude_ = attitude;

	osg::Vec3 rayVector = attitude_ *  osg::Vec3(1,0,0);

	// create ray
	dGeomID ray = dCreateRay(raySpace_, SENSOR_RANGE);
	// position ray
	dGeomRaySet(ray, position_.x(), position_.y(), position_.z(),
			rayVector.x(), rayVector.y(), rayVector.z());

	// prepare collision data structure
	RayTrace data;

	// ignore sensor bodies
	data.ignoreGeoms.insert(data.ignoreGeoms.end(),
					sensorGeoms_.begin(), sensorGeoms_.end());

	data.isColliding = false;

	// perform collision
	dSpaceCollide2((dGeomID)raySpace_, (dGeomID)odeSpace_, (void*)&data,
			LowResCameraSensor::collisionCallback);

	dGeomDestroy(ray);
}


}



