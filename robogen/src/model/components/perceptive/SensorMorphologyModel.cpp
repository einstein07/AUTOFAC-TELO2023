/*
 * @(#) SensorMorphologyModel.cpp   1.0   Jun 01, 2022
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#include "model/components/perceptive/SensorMorphologyModel.h"


namespace robogen {

const float SensorMorphologyModel::MASS = inGrams(2); //todo verify!
const float SensorMorphologyModel::SENSOR_BASE_WIDTH = inMm(34);
const float SensorMorphologyModel::SENSOR_BASE_THICKNESS = inMm(1.5);

const float SensorMorphologyModel::SENSOR_PLATFORM_WIDTH = inMm(23.3);
const float SensorMorphologyModel::SENSOR_PLATFORM_HEIGHT = inMm(23);
const float SensorMorphologyModel::SENSOR_PLATFORM_THICKNESS = inMm(5.5);

const float SensorMorphologyModel::SENSOR_DISPLACEMENT = inMm(6);

SensorMorphologyModel::SensorMorphologyModel(dWorldID odeWorld, dSpaceID odeSpace,
		std::string id) :
		PerceptiveComponent(odeWorld, odeSpace, id) {
}

SensorMorphologyModel::SensorMorphologyModel(dWorldID odeWorld, dSpaceID odeSpace, dSpaceID robotSpace,
		std::string id) :
		PerceptiveComponent(odeWorld, odeSpace, robotSpace, id) {
}

SensorMorphologyModel::~SensorMorphologyModel() {

}

bool SensorMorphologyModel::initModel() {


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

	this->sensor_.reset(new SensorMorphology(this->getODECollisionSpace(),
			this->getBodies(), this->getId()));

	return true;

}

boost::shared_ptr<SimpleBody> SensorMorphologyModel::getRoot() {
	return sensorRoot_;
}

boost::shared_ptr<SimpleBody> SensorMorphologyModel::getSlot(unsigned int /*i*/) {
	return sensorRoot_;
}

osg::Vec3 SensorMorphologyModel::getSlotPosition(unsigned int i) {

	if (i > 1) {
		std::cout << "[SensorMorphologyModel] Invalid slot: " << i << std::endl;
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

osg::Vec3 SensorMorphologyModel::getSlotAxis(unsigned int i) {

	if (i > 1) {
		std::cout << "[SensorMorphologyModel] Invalid slot: " << i << std::endl;
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

osg::Vec3 SensorMorphologyModel::getSlotOrientation(unsigned int i) {

	if (i > 1) {
		std::cout << "[SensorMorphologyModel] Invalid slot: " << i << std::endl;
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

void SensorMorphologyModel::getSensors(
		std::vector<boost::shared_ptr<Sensor> >& sensors) {
	sensor_->getSensors(sensors);
}

void SensorMorphologyModel::updateSensors(boost::shared_ptr<Environment>& env) {
	//std::cout << "Updating color sensor. . ." << std::endl;
	// Axis
	osg::Quat quat = this->sensorRoot_->getAttitude();

	osg::Vec3 curPos = this->sensorRoot_->getPosition();
	osg::Vec3 axis(1, 0, 0);
	osg::Vec3 sensorPos = curPos + quat * axis * SENSOR_DISPLACEMENT;
	this->sensor_->update(sensorPos, this->sensorRoot_->getAttitude(), env);
	//std::cout << "Done updating color sensor." << std::endl;
}

}
