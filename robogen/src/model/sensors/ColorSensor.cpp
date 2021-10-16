/*
 * @(#) ColorSensor.cpp   1.0   Aug 19, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#include "ColorSensor.h"

#include <algorithm>
#include <iostream>
#include <cmath>

namespace robogen {

const float ColorSensor::SENSOR_RANGE = /*0.255*/1;

ColorSensor::ColorSensor(dSpaceID odeSpace,
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
	sensors_.push_back(
				boost::shared_ptr<Sensor>(new IrSensorElement(baseLabel,
						IrSensorElement::IR)));
	//Sense if object is a robot
	sensors_.push_back(
			boost::shared_ptr<Sensor>(new ColorSensorElement(baseLabel,
					ColorSensorElement::ROBOT)));
	//Sense if object is a resource
	sensors_.push_back(
			boost::shared_ptr<Sensor>(new ColorSensorElement(baseLabel,
					ColorSensorElement::RESOURCE)));
	//Sense size of resource
	sensors_.push_back(
			boost::shared_ptr<Sensor>(new ColorSensorElement(baseLabel,
					ColorSensorElement::RESOURCESIZE)));
	//Sense if object is the target area
	/**	sensors_.push_back(
			boost::shared_ptr<Sensor>(new ColorSensorElement(baseLabel,
					ColorSensorElement::TARGETAREA)));*/
	//Sense if object is a WALL
			sensors_.push_back(
				boost::shared_ptr<Sensor>(new ColorSensorElement(baseLabel,
						ColorSensorElement::WALL)));
}

ColorSensor::~ColorSensor() {
	dSpaceDestroy(raySpace_);
}



struct RayTrace {
	std::vector<dGeomID> ignoreGeoms;
	bool isColliding;
	osg::Vec3 collisionPoint;
	ObjectData *objData;
};

/**void ColorSensor::collisionCallback(void *data, dGeomID o1, dGeomID o2){
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
		//std::cout << "Getting pointer to geom data" << std::endl;
		r->objData = (ObjectData*)dGeomGetData(o1);
		if(r->objData == NULL){
			//std::cout << "**Geom o1** is not either one of the objects of interest." << std::endl;
			//std::cout << "**Geom o2** is an object of interest." << std::endl;
			r->objData = (ObjectData*)dGeomGetData(o2);
		}
		//if(r->objData != NULL){
		//	std::cout << "Managed to get a valid pointer." <<std::endl;
		//}
	}
}*/

void ColorSensor::collisionCallback(void *data, dGeomID o1, dGeomID o2) {

	RayTrace *r = (RayTrace*) data;


	if (dGeomIsSpace (o1) || dGeomIsSpace (o2)) {
		/**if (dGeomIsSpace (o1))
			std::cout <<"Space ID: " << o2 << ". Total number of geoms in O1: " << dSpaceGetNumGeoms((dSpaceID)o1)
				<< std::endl;
		if (dGeomIsSpace (o2))
			std::cout << "Space ID: " << o2 << ". Total number of geoms in O2: " << dSpaceGetNumGeoms((dSpaceID)o2)
						<< std::endl;*/
		//std::cout << "Colliding a space with something" << std::endl;
		// colliding a space with something :
		dSpaceCollide2 (o1, o2, data, collisionCallback);
	}
	else{
		// ignore what needs ignoring
		if (std::find(r->ignoreGeoms.begin(),r->ignoreGeoms.end(),
				o1) != r->ignoreGeoms.end() ||
				std::find(r->ignoreGeoms.begin(),r->ignoreGeoms.end(),
						o2) != r->ignoreGeoms.end()){
			//std::cout << "Ignoring what needs to be ignored." << std::endl;
			return;
		}

		// if colliding get collision point
		dContact contact;

		if (dCollide(o1,o2,1,&contact.geom,sizeof(contact))) {
			r->collisionPoint =  osg::Vec3(contact.geom.pos[0],
					contact.geom.pos[1],
					contact.geom.pos[2]);
			r->isColliding = true;
			//std::cout << "Getting pointer to geom data" << std::endl;
			r->objData = (ObjectData*)dGeomGetData(o1);
			if(r->objData == NULL){
				//std::cout << "**Geom o1** is not either one of the objects of interest." << std::endl;
				//std::cout << "**Geom o2** is an object of interest." << std::endl;
				r->objData = (ObjectData*)dGeomGetData(o2);
			}
		}

	}

}

void ColorSensor::update(const osg::Vec3& position, const osg::Quat& attitude, boost::shared_ptr<Environment>& env) {
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
					ColorSensor::collisionCallback);

	float distance = /**SENSOR_RANGE*/ 0.0;

	// ray should be capped at SENSOR_RANGE, but just to make sure we don't
	// allow bigger values here
	if(data.isColliding &&
			(data.collisionPoint - position_).length() < SENSOR_RANGE) {
		distance = (data.collisionPoint - position_).length();

		//std::cout << "pos: " << position_.x() << " " << position_.y() <<  " " << position_.z() << std::endl;
		//std::cout << "ray: " << rayVector.x() << " " << rayVector.y() <<  " " << rayVector.z() << std::endl;
		//std::cout << "collision: " << data.collisionPoint.x() << " " << data.collisionPoint.y() <<  " " << data.collisionPoint.z() << std::endl;

	}

	int value = -1;

	// ray should be capped at SENSOR_RANGE, but just to make sure we don't
	// allow bigger values here
	if(data.isColliding && data.objData != NULL) {
		if (data.objData->isRobot){
			value = ColorSensorElement::Type::ROBOT;
		}
		else if (data.objData->isResource){
			value = ColorSensorElement::Type::RESOURCE;
		}
		/**else if (data.objData->isTargetArea){
			value = ColorSensorElement::Type::TARGETAREA;
		}*/
		else if (data.objData->isWall){
			value = ColorSensorElement::Type::WALL;
		}

	}

	dGeomDestroy(ray);
	// want 0 when nothing is seen
	sensors_[0]->updateValue( /**1.0 - (distance / SENSOR_RANGE)*/ distance );
	//Either nothing was detected or the detected object is not a robot nor a resorce
	if (value == -1){
		sensors_[1]->updateValue(0.0);
		sensors_[2]->updateValue(0.0);
		sensors_[3]->updateValue(0.0);
		sensors_[4]->updateValue(0.0);
		//sensors_[5] -> updateValue(0.0);
	}

	// Robot detected
	else if ( value == ColorSensorElement::ROBOT ){
		sensors_[1] -> updateValue(1.0);
		sensors_[2] -> updateValue(0.0);
		sensors_[3] -> updateValue(0.0);
		sensors_[4] -> updateValue(0.0);
		//sensors_[5] -> updateValue(0.0);
		/**
		 * Update robot object id
		 */
		boost::dynamic_pointer_cast<ColorSensorElement>(sensors_[1])->updateObjectId(data.objData->objectId);
	}

	// Resource detected
	else if ( value == ColorSensorElement::RESOURCE ){
		sensors_[1] -> updateValue(0.0);
		sensors_[2] -> updateValue(1.0);

		//--------------------------------------------------------
		double size = 0.0;

		boost::shared_ptr<BoxResource> resource = env->getResources()[data.objData->objectId];
		int numRobotsAttached = resource->getNumberPushingRobots() + 1; //plus this robot
		int numRobotsRequired = resource->getSize();
		//std::cout << "Number of robots attached to this resource: " << resource->getNumberPushingRobots() << std::endl;
		//std::cout << "Number of robots required to push this resource: " << resource->getSize() << std::endl;
		if ((numRobotsAttached < numRobotsRequired) || (numRobotsAttached == numRobotsRequired)){
			//std::cout << "WITHIN" << std::endl;
			size = ((double)numRobotsAttached / (double)numRobotsRequired);
		}

		//--------------------------------------------------------
		sensors_[3] -> updateValue(size);

		sensors_[4] -> updateValue(0.0);

		//sensors_[5] -> updateValue(0.0);
		/**
		 * Update resource object id - both sensors will have to point to the same
		 * resource object id to ensure correctness of sensor object detection
		 */
		boost::dynamic_pointer_cast<ColorSensorElement>(sensors_[2])->updateObjectId(data.objData->objectId);
		boost::dynamic_pointer_cast<ColorSensorElement>(sensors_[3])->updateObjectId(data.objData->objectId);

	}

	// Target area is detected
	/**else if ( value == ColorSensorElement::TARGETAREA ){
		//std::cout << "TARGET AREA DETECTED" << std::endl;
		sensors_[1]->updateValue(0.0);
		sensors_[2]->updateValue(0.0);
		sensors_[3]->updateValue(0.0);
		sensors_[4] -> updateValue(1.0);
		sensors_[5] -> updateValue(0.0);
	}*/

	// Wall detected
	else if ( value == ColorSensorElement::WALL ){
		sensors_[1] -> updateValue(0.0);
		sensors_[2] -> updateValue(0.0);
		sensors_[3] -> updateValue(0.0);
		sensors_[4] -> updateValue(1.0);
		//sensors_[5] -> updateValue(1.0);
	}

}


}



