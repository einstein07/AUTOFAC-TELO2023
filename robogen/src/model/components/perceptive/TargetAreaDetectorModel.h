/*
 * @(#) TargetAreaDetectorModel.h   1.0   Oct 16, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef ROBOGEN_TARGET_AREA_DETECTOR_MODEL_H_
#define ROBOGEN_TARGET_AREA_DETECTOR_MODEL_H_


#include <boost/shared_ptr.hpp>
#include "model/PerceptiveComponent.h"
#include "model/sensors/TargetAreaDetector.h"

namespace robogen {

class TargetAreaDetectorModel: public PerceptiveComponent {

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
	TargetAreaDetectorModel(dWorldID odeWorld, dSpaceID odeSpace, std::string id);
	TargetAreaDetectorModel(dWorldID odeWorld, dSpaceID odeSpace, dSpaceID robotSpace, std::string id);

	virtual ~TargetAreaDetectorModel();

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

	boost::shared_ptr<TargetAreaDetector> sensor_;

};

}



#endif /* ROBOGEN_TARGET_AREA_DETECTOR_MODEL_H_ */
