/*
 * @(#) LowResCameraSensor.h   1.0   Aug 19, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef ROBOGEN_LOWRESCAMERA_SENSOR_H_
#define ROBOGEN_LOWRESCAMERA_SENSOR_H_

#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>

#include "Robogen.h"
#include "model/sensors/Sensor.h"
#include "model/SimpleBody.h"
#include "SensorGroup.h"

namespace robogen {



class LowResCameraSensor : public Sensor {

public:

	static const float SENSOR_RANGE;

	/**
	 * Initializes a low res camera sensor
	 */
	LowResCameraSensor(dSpaceID odeSpace,
			std::vector<boost::shared_ptr<SimpleBody> > sensorBodies,
			std::string baseLabel);

	/**
	 * Destructor
	 */
	virtual ~LowResCameraSensor();

	/**
	 * Update the low res camera sensor
	 */
	void update(const osg::Vec3& position, const osg::Quat& attitude);

	/**
	 * Callback for collision handling between rays and the ODE space
	 */
	static void collisionCallback(void *data, dGeomID o1, dGeomID o2);

	const std::string &getBaseLabel() { return baseLabel_; }

	/**
	 * @return objectId_
	 */
	const int getObjectId(){return objectId_;}
	void updateObjectId(int objectId){objectId_ = objectId;}

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
	 * Position of the low res camera sensor
	 */
	osg::Vec3 position_;

	/**
	 * Attitude of the low res camera sensor
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
	 * ODE bodies of the low res camera sensor
	 */
	std::vector<dGeomID> sensorGeoms_;

	std::string baseLabel_;

	int objectId_;

};

}

#endif /* ROBOGEN_LOWRESCAMERA_SENSOR_H_ */
