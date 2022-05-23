/*
 * @(#) LowResCameraSensorModel.cpp   1.0   Aug 19, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#include "model/components/perceptive/LowResCameraSensorModel.h"


namespace robogen {

const float LowResCameraSensorModel::MASS = inGrams(2); //todo verify!
const float LowResCameraSensorModel::SENSOR_BASE_WIDTH = inMm(34);
const float LowResCameraSensorModel::SENSOR_BASE_THICKNESS = inMm(1.5);

const float LowResCameraSensorModel::SENSOR_PLATFORM_WIDTH = inMm(23.3);
const float LowResCameraSensorModel::SENSOR_PLATFORM_HEIGHT = inMm(23);
const float LowResCameraSensorModel::SENSOR_PLATFORM_THICKNESS = inMm(5.5);

const float LowResCameraSensorModel::SENSOR_DISPLACEMENT = inMm(6);

LowResCameraSensorModel::LowResCameraSensorModel(dWorldID odeWorld, dSpaceID odeSpace,
		dSpaceID robotSpace, std::string id) :
		PerceptiveComponent(odeWorld, odeSpace, robotSpace, id) {
}

LowResCameraSensorModel::~LowResCameraSensorModel() {

}

bool LowResCameraSensorModel::initModel() {


	//just dividing the mass by 2 for each component, is there a better way to do this?
	sensorRoot_ = this->addBox(MASS/2.0, osg::Vec3(0, 0, 0),
			SENSOR_BASE_THICKNESS, SENSOR_BASE_WIDTH, SENSOR_BASE_WIDTH,
			B_SENSOR_BASE_ID);

	boost::shared_ptr<SimpleBody> platform = this->addBox(MASS/2.0,
			osg::Vec3(SENSOR_BASE_THICKNESS/2.0 +
					  SENSOR_PLATFORM_THICKNESS/2.0, 0, 0),
			SENSOR_PLATFORM_THICKNESS, SENSOR_PLATFORM_WIDTH,
			SENSOR_PLATFORM_HEIGHT, B_SENSOR_PLATFORM_ID);

	this->fixBodies(sensorRoot_, platform);

	this->sensor_.reset(new LowResCameraSensor(this->getODECollisionSpace(),
			this->getBodies(), this->getId()));

	return true;

}

boost::shared_ptr<SimpleBody> LowResCameraSensorModel::getRoot() {
	return sensorRoot_;
}

boost::shared_ptr<SimpleBody> LowResCameraSensorModel::getSlot(unsigned int /*i*/) {
	return sensorRoot_;
}

osg::Vec3 LowResCameraSensorModel::getSlotPosition(unsigned int i) {

	if (i > 1) {
		std::cout << "[LowResCameraSensorModel] Invalid slot: " << i << std::endl;
		assert(i <= 1);
	}

	osg::Vec3 slotPos;
	if (i == SLOT_A) {

		osg::Vec3 curPos = this->sensorRoot_->getPosition();
		osg::Vec3 slotAxis = this->getSlotAxis(i);
		slotPos = curPos + slotAxis * (SENSOR_BASE_THICKNESS / 2);

	}

	return slotPos;

}

osg::Vec3 LowResCameraSensorModel::getSlotAxis(unsigned int i) {

	if (i > 1) {
		std::cout << "[LowResCameraSensorModel] Invalid slot: " << i << std::endl;
		assert(i <= 1);
	}

	osg::Quat quat;
	osg::Vec3 axis;

	if (i == SLOT_A) {

		quat = this->sensorRoot_->getAttitude();
		axis.set(-1, 0, 0);

	}

	return quat * axis;

}

osg::Vec3 LowResCameraSensorModel::getSlotOrientation(unsigned int i) {

	if (i > 1) {
		std::cout << "[IrSensorModel] Invalid slot: " << i << std::endl;
		assert(i <= 1);
	}

	osg::Quat quat;
	osg::Vec3 axis;

	if (i == SLOT_A) {

		quat = this->sensorRoot_->getAttitude();
		axis.set(0, 1, 0);

	}
	return quat * axis;

}

void LowResCameraSensorModel::getSensors(
		std::vector<boost::shared_ptr<Sensor> >& sensors) {
	sensors.resize(1);
	sensors[0] = sensor_;
}

void LowResCameraSensorModel::updateSensors(boost::shared_ptr<Environment>& env) {
	std::cout << "Updating low res camera sensor. . ." << std::endl;
	// Axis
	osg::Quat quat = this->sensorRoot_->getAttitude();

	osg::Vec3 curPos = this->sensorRoot_->getPosition();
	osg::Vec3 axis(1, 0, 0);
	osg::Vec3 sensorPos = curPos + quat * axis * SENSOR_DISPLACEMENT;
	this->sensor_->update(sensorPos, this->sensorRoot_->getAttitude());
	double value = 0.0;
	if(sensor_->getObjectId() != -1){
		boost::shared_ptr<BoxResource> resource = env->getResources()[sensor_->getObjectId()];
		int numRobotsAttached = resource->getNumberPushingRobots() + 1; //plus this robot
		int numRobotsRequired = resource->getSize();
		if (numRobotsAttached <= numRobotsRequired){
			value = (double)(numRobotsAttached / numRobotsRequired);
		}

	}
	this->sensor_->updateValue(value);
	std::cout << "Done updating low res camera sensor." << std::endl;

}

}
