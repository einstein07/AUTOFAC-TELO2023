/*
 * @(#) SensorMorphology.h   1.0   June 01, 2022
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#ifndef ROBOGEN_SENSOR_MORPHOLOGY_H_
#define ROBOGEN_SENSOR_MORPHOLOGY_H_

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
class SensorElement : public Sensor {
public:

	enum Type {
		IR,
		ROBOT,
		RESOURCET1,
		RESOURCET2,
		RESOURCET3,
		RESOURCET4,
		RESOURCET5,
		WALL
	};

	inline SensorElement(std::string baseLabel, Type type) :
			Sensor(baseLabel + (
								(type == IR) ?  "-IR" :
								(type == ROBOT) ?  "-Robot" :
								(type == RESOURCET1)? "-Resource-Type-1" :
								(type == RESOURCET2)? "-Resource-Type-2" :
								(type == RESOURCET3)? "-Resource-Type-3" :
								(type == RESOURCET4)? "-Resource-Type-4" :
								(type == RESOURCET5)? "-Resource-Type-5" : "-Wall"
								)),
			baseLabel_(baseLabel), type_(type), objectId_(-1), value_(0.0) {
	}

	inline Type getType() { return type_; }
	inline const std::string &getBaseLabel() { return baseLabel_; }
	inline int getObjectId() { return objectId_; }
	inline void updateObjectId(int objectId) { objectId_ = objectId; }
	inline bool isActive(){return (value_ < 0.5);}
	inline double getValue(){return value_;}
	inline void setValue(double value){value_ = value;}
private:
	std::string baseLabel_;
	Type type_;
	int objectId_;
	double value_;
};
//----------------------------------------------------------------------
//----------------------------------------------------------------------
class SensorMorphology : public SensorGroup {

public:

	static const float SENSOR_RANGE;

	/**
	 * Initializes a sensor morphology
	 */
	SensorMorphology(dSpaceID odeSpace,
			std::vector<boost::shared_ptr<SimpleBody> > sensorBodies,
			std::string baseLabel);

	/**
	 * Destructor
	 */
	virtual ~SensorMorphology();

	/**
	 * Update the sensor
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
	 * Position of the sensor
	 */
	osg::Vec3 position_;

	/**
	 * Attitude of the sensor
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
	 * ODE bodies of the sensor elements
	 */
	std::vector<dGeomID> sensorGeoms_;

};

}

#endif /* ROBOGEN_SENSOR_MORPHOLOGY_H_ */
