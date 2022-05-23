/*
 * @(#) ColorSensorModel.cpp   1.0   Aug 19, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#include "model/components/perceptive/ColorSensorModel.h"


namespace robogen {

const float ColorSensorModel::MASS = inGrams(2); //todo verify!
const float ColorSensorModel::SENSOR_BASE_WIDTH = inMm(34);
const float ColorSensorModel::SENSOR_BASE_THICKNESS = inMm(1.5);

const float ColorSensorModel::SENSOR_PLATFORM_WIDTH = inMm(23.3);
const float ColorSensorModel::SENSOR_PLATFORM_HEIGHT = inMm(23);
const float ColorSensorModel::SENSOR_PLATFORM_THICKNESS = inMm(5.5);

const float ColorSensorModel::SENSOR_DISPLACEMENT = inMm(6);

ColorSensorModel::ColorSensorModel(dWorldID odeWorld, dSpaceID odeSpace,
		std::string id) :
		PerceptiveComponent(odeWorld, odeSpace, id) {
}

ColorSensorModel::ColorSensorModel(dWorldID odeWorld, dSpaceID odeSpace, dSpaceID robotSpace,
		std::string id) :
		PerceptiveComponent(odeWorld, odeSpace, robotSpace, id) {
}

ColorSensorModel::~ColorSensorModel() {

}

bool ColorSensorModel::initModel() {


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

	this->sensor_.reset(new ColorSensor(this->getODECollisionSpace(),
			this->getBodies(), this->getId()));

	return true;

}

boost::shared_ptr<SimpleBody> ColorSensorModel::getRoot() {
	return sensorRoot_;
}

boost::shared_ptr<SimpleBody> ColorSensorModel::getSlot(unsigned int /*i*/) {
	return sensorRoot_;
}

osg::Vec3 ColorSensorModel::getSlotPosition(unsigned int i) {

	if (i > 1) {
		std::cout << "[IrSensorModel] Invalid slot: " << i << std::endl;
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

osg::Vec3 ColorSensorModel::getSlotAxis(unsigned int i) {

	if (i > 1) {
		std::cout << "[IrSensorModel] Invalid slot: " << i << std::endl;
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

osg::Vec3 ColorSensorModel::getSlotOrientation(unsigned int i) {

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

void ColorSensorModel::getSensors(
		std::vector<boost::shared_ptr<Sensor> >& sensors) {
	sensor_->getSensors(sensors);
}

void ColorSensorModel::updateSensors(boost::shared_ptr<Environment>& env) {
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
