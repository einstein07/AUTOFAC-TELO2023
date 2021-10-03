/*
 * @(#) LowResCameraSensorModel.h   1.0   Aug 19, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef ROBOGEN_LOW_RES_CAMERA_SENSOR_MODEL_H_
#define ROBOGEN_LOW_RES_CAMERA_SENSOR_MODEL_H_


#include <boost/shared_ptr.hpp>
#include "model/PerceptiveComponent.h"
#include "model/sensors/LowResCameraSensor.h"

namespace robogen {

class LowResCameraSensorModel: public PerceptiveComponent {

public:

	static const float MASS;
	static const float SENSOR_BASE_WIDTH;
	static const float SENSOR_BASE_THICKNESS;
	static const float SENSOR_PLATFORM_WIDTH;
	static const float SENSOR_PLATFORM_HEIGHT;
	static const float SENSOR_PLATFORM_THICKNESS;

	static const float SENSOR_DISPLACEMENT;

	static const unsigned int SLOT_A = 0;

	static const unsigned int B_SENSOR_BASE_ID = 0;
	static const unsigned int B_SENSOR_PLATFORM_ID = 1;

	LowResCameraSensorModel(dWorldID odeWorld, dSpaceID odeSpace,
			dSpaceID robotSpace, std::string id);

	virtual ~LowResCameraSensorModel();

	virtual bool initModel();

	virtual boost::shared_ptr<SimpleBody> getRoot();

	virtual boost::shared_ptr<SimpleBody> getSlot(unsigned int i);

	virtual osg::Vec3 getSlotPosition(unsigned int i);

	virtual osg::Vec3 getSlotOrientation(unsigned int i);

	virtual osg::Vec3 getSlotAxis(unsigned int i);

	virtual void getSensors(std::vector<boost::shared_ptr<Sensor> >& sensors);

	virtual void updateSensors(boost::shared_ptr<Environment>& env);

private:

	boost::shared_ptr<SimpleBody> sensorRoot_;

	boost::shared_ptr<LowResCameraSensor> sensor_;

};

}



#endif /* ROBOGEN_LOW_RES_CAMERA_SENSOR_MODEL_H_ */
