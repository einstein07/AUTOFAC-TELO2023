/*
 * @(#) ColorSensor.h   1.0   Aug 19, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#ifndef ROBOGEN_COLOR_SENSOR_H_
#define ROBOGEN_COLOR_SENSOR_H_

#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>

#include "Robogen.h"
#include "model/sensors/Sensor.h"
#include "model/SimpleBody.h"
#include "SensorGroup.h"
#include "IrSensor.h"
#include "scenario/Environment.h"

namespace robogen {

//----------------------------------------------------------------------
//----------------------------------------------------------------------
class ColorSensorElement : public Sensor {
public:

	enum Type {
		ROBOT,
		RESOURCE,
		RESOURCESIZE
	};

	inline ColorSensorElement(std::string baseLabel, Type type) :
			Sensor(baseLabel + ((type == ROBOT) ?  "-Robot" :
										"-Resource")),
			baseLabel_(baseLabel), type_(type), objectId_(-1) {
	}

	inline Type getType() { return type_; }
	inline const std::string &getBaseLabel() { return baseLabel_; }
	inline int getObjectId() { return objectId_; }
	inline void updateObjectId(int objectId) { objectId_ = objectId; }
private:
	std::string baseLabel_;
	Type type_;
	int objectId_;
};
//----------------------------------------------------------------------
//----------------------------------------------------------------------
class ColorSensor : public SensorGroup {

public:

	static const float SENSOR_RANGE;

	/**
	 * Initializes a color sensor
	 */
	ColorSensor(dSpaceID odeSpace,
			std::vector<boost::shared_ptr<SimpleBody> > sensorBodies,
			std::string baseLabel);

	/**
	 * Destructor
	 */
	virtual ~ColorSensor();

	/**
	 * Update the color sensor
	 */
	void update(const osg::Vec3& position, const osg::Quat& attitude, boost::shared_ptr<Environment>& env);

	/**
	 * Callback for collision handling between rays and the ODE space
	 */
	static void collisionCallback(void *data, dGeomID o1, dGeomID o2);

private:
	/**
	 * Calculates intensity at light sensor from angle, light's intensity,
	 * and distance. This is the function to be modified according to
	 * light sensor calibration.
	 * @param angle
	 * @param lightIntensity
	 * @param distance
	 * @return intensity
	 */
	static double getIntensity(double angle, double lightIntensity,
			double distance);

	/**
	 * Ode collision space
	 */
	dSpaceID odeSpace_;

	/**
	 *
	 * Per Robot collision space
	 */
	//dSpaceID robotSpace_;

	/**
	 * Position of the Color sensor
	 */
	osg::Vec3 position_;

	/**
	 * Attitude of the Color sensor
	 */
	osg::Quat attitude_;

	/**
	 * Space for rays
	 */
	dSpaceID raySpace_;

	/**
	 * Output of the last read
	 */
	float lastReadOutput_;

	/**
	 * ODE bodies of the Color sensor
	 */
	std::vector<dGeomID> sensorGeoms_;

};

}

#endif /* ROBOGEN_COLOR_SENSOR_H_ */
