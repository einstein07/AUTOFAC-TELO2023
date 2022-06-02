/*
 * @(#) SensorMorphologyRenderModel.h   1.0   Jun 01, 2022
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef ROBOGEN_SENSOR_MORPHOLOGY_RENDER_MODEL_H_
#define ROBOGEN_SENSOR_MORPHOLOGY_RENDER_MODEL_H_

#include "model/components/perceptive/SensorMorphologyModel.h"
#include "render/RenderModel.h"

namespace robogen {

class Mesh;

class SensorMorphologyRenderModel: public RenderModel {

public:

	SensorMorphologyRenderModel(boost::shared_ptr<SensorMorphologyModel> model);

	virtual ~SensorMorphologyRenderModel();

	virtual bool initRenderModel();

	void showDebugView();

	virtual void setColor(osg::Vec4 color);

private:

	boost::shared_ptr<Mesh> partA_;

};

}



#endif /* ROBOGEN_SENSOR_MORPHOLOGY_RENDER_MODEL_H_ */
