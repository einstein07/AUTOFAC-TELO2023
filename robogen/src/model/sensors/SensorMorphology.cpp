/*
 * @(#) SensorMorphology.cpp   1.0   June 01, 2022
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#include "SensorMorphology.h"

#include <algorithm>
#include <iostream>
#include <cmath>

namespace robogen {

const float SensorMorphology::SENSOR_RANGE = /*0.255*/5;

SensorMorphology::SensorMorphology(dSpaceID odeSpace,
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

	//[0]Distance to detected object
	sensors_.push_back(
				boost::shared_ptr<Sensor>(new IrSensorElement(baseLabel,
						IrSensorElement::IR)));
	//[1]Sense if object is a robot
	sensors_.push_back(
			boost::shared_ptr<Sensor>(new SensorElement(baseLabel,
					SensorElement::ROBOT)));
	//[2]Sense if object is a type 1 resource
	sensors_.push_back(
			boost::shared_ptr<Sensor>(new SensorElement(baseLabel,
					SensorElement::RESOURCET1)));
	//[3]Sense if object is a type 2 resource
	sensors_.push_back(
		boost::shared_ptr<Sensor>(new SensorElement(baseLabel,
				SensorElement::RESOURCET2)));
	//[4]Sense if object is a type 3 resource
	sensors_.push_back(
			boost::shared_ptr<Sensor>(new SensorElement(baseLabel,
					SensorElement::RESOURCET3)));
	//[5]Sense if object is a type 4 resource
	sensors_.push_back(
			boost::shared_ptr<Sensor>(new SensorElement(baseLabel,
					SensorElement::RESOURCET4)));
	//[6]Sense if object is a type 5 resource
	sensors_.push_back(
			boost::shared_ptr<Sensor>(new SensorElement(baseLabel,
					SensorElement::RESOURCET5)));
	//[7]Sense if object is a WALL
	sensors_.push_back(
		boost::shared_ptr<Sensor>(new SensorElement(baseLabel,
				SensorElement::WALL)));
}

SensorMorphology::~SensorMorphology() {
	dSpaceDestroy(raySpace_);
}



struct RayTrace {
	std::vector<dGeomID> ignoreGeoms;
	bool isColliding;
	osg::Vec3 collisionPoint;
	ObjectData *objData;
};

void SensorMorphology::collisionCallback(void *data, dGeomID o1, dGeomID o2) {

	RayTrace *r = (RayTrace*) data;


	if (dGeomIsSpace (o1) || dGeomIsSpace (o2)) {
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

	void SensorMorphology::update(const osg::Vec3& position, const osg::Quat& attitude, boost::shared_ptr<Environment>& env) {
		this->position_ = position;
		this->attitude_ = attitude;
		osg::Vec3 rayVector = attitude_ *  osg::Vec3(1,0,0);

		// create ray
		dGeomID ray = dCreateRay(raySpace_, SENSOR_RANGE);
		// position ray
		dGeomRaySet(ray, position_.x(), position_.y(), position_.z(),
				rayVector.x(), rayVector.y(), rayVector.z());
		//*******************************************
		// prepare collision data structure
		//*******************************************
		RayTrace data;

		// ignore sensor bodies
		data.ignoreGeoms.insert(data.ignoreGeoms.end(),
						sensorGeoms_.begin(), sensorGeoms_.end());

		data.isColliding = false;
		data.objData = 0;
		// perform collision
		dSpaceCollide2((dGeomID)raySpace_, (dGeomID)odeSpace_, (void*)&data,
				SensorMorphology::collisionCallback);

		float distance = /**SENSOR_RANGE*/ 0.0;

		// ray should be capped at SENSOR_RANGE, but just to make sure we don't
		// allow bigger values here
		if(data.isColliding &&
				(data.collisionPoint - position_).length() < SENSOR_RANGE) {
			distance = (data.collisionPoint - position_).length();
		}

		int value = -1;

		// ray should be capped at SENSOR_RANGE, but just to make sure we don't
		// allow bigger values here
		if(data.isColliding && data.objData != 0) {
			if (data.objData->isRobot){
				value = SensorElement::Type::ROBOT;
			}
			else if (data.objData->isResource){
				//Assume it is a type 1 resource for now
				value = SensorElement::Type::RESOURCET1;
			}
			else if (data.objData->isWall){
				value = SensorElement::Type::WALL;
			}
		}
		dGeomDestroy(ray);
		// want 0 when nothing is seen
		sensors_[SensorElement::IR]->updateValue( /**1.0 - (distance / SENSOR_RANGE)*/ distance );
		//Either nothing was detected or the detected object is not a robot nor a resorce
		if (value == -1){
			sensors_[SensorElement::ROBOT]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET1]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET2]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET3]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET4]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET5]->updateValue(0.0);
			sensors_[SensorElement::WALL]->updateValue(0.0);
		}

		// Robot detected
		else if ( value == SensorElement::ROBOT ){
			sensors_[SensorElement::ROBOT]->updateValue(1.0);
			sensors_[SensorElement::RESOURCET1]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET2]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET3]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET4]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET5]->updateValue(0.0);
			sensors_[SensorElement::WALL]->updateValue(0.0);
			/**
			 * Update robot object id
			 */
			boost::dynamic_pointer_cast<SensorElement>(sensors_[SensorElement::ROBOT])
					->updateObjectId(data.objData->objectId);

		}

		// Resource detected
		else if ( value == SensorElement::RESOURCET1 ){
			sensors_[SensorElement::ROBOT]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET1]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET2]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET3]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET4]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET5]->updateValue(0.0);
			sensors_[SensorElement::WALL]->updateValue(0.0);

			//--------------------------------------------------------

			int type = 1;
			int objectId = data.objData->objectId;
			if ( objectId < 0 || objectId >= env->getResources().size() ){
				std::cerr << "[COLOR SENSOR] - Box resource with object id: ** " << objectId << " could **NOT** be retrieved by robot: " << std::endl;

			}
			else{
				boost::shared_ptr<BoxResource> resource = env->getResources()[objectId];
				// Should not happen
				if (resource != NULL){
					type = resource->getType();
					if (type == 1){
						sensors_[SensorElement::RESOURCET1]->updateValue(1.0);
						boost::dynamic_pointer_cast<SensorElement>(sensors_[SensorElement::RESOURCET1])
								->updateObjectId(data.objData->objectId);
					}
					else if (type == 2){
						sensors_[SensorElement::RESOURCET2]->updateValue(1.0);
						boost::dynamic_pointer_cast<SensorElement>(sensors_[SensorElement::RESOURCET2])
								->updateObjectId(data.objData->objectId);
					}
					else if (type == 3){
						sensors_[SensorElement::RESOURCET3]->updateValue(1.0);
						boost::dynamic_pointer_cast<SensorElement>(sensors_[SensorElement::RESOURCET3])
								->updateObjectId(data.objData->objectId);
					}
					else if (type == 4){
						sensors_[SensorElement::RESOURCET4]->updateValue(1.0);
						boost::dynamic_pointer_cast<SensorElement>(sensors_[SensorElement::RESOURCET4])
								->updateObjectId(data.objData->objectId);
					}
					else if (type == 5){
						sensors_[SensorElement::RESOURCET5]->updateValue(1.0);
						boost::dynamic_pointer_cast<SensorElement>(sensors_[SensorElement::RESOURCET5])->updateObjectId(data.objData->objectId);
					}
					else{
						std::cerr << "[Sensor element] - Box resource of unknown type: "<< type << std::endl;
					}
				}

			}

			//--------------------------------------------------------

		}
		// Wall detected
		else if ( value == SensorElement::WALL ){
			sensors_[SensorElement::ROBOT]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET1]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET2]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET3]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET4]->updateValue(0.0);
			sensors_[SensorElement::RESOURCET5]->updateValue(0.0);
			sensors_[SensorElement::WALL]->updateValue(1.0);
			boost::dynamic_pointer_cast<SensorElement>(sensors_[SensorElement::WALL])->updateObjectId(data.objData->objectId);

		}
	}
}


