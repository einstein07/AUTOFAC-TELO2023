/*
 * @(#) ColorSensorRenderModel.h   1.0   Aug 22, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef ROBOGEN_COLOR_SENSOR_RENDER_MODEL_H_
#define ROBOGEN_COLOR_SENSOR_RENDER_MODEL_H_

#include "model/components/perceptive/ColorSensorModel.h"
#include "render/RenderModel.h"

namespace robogen {

class Mesh;

class ColorSensorRenderModel: public RenderModel {

public:

	ColorSensorRenderModel(boost::shared_ptr<ColorSensorModel> model);

	virtual ~ColorSensorRenderModel();

	virtual bool initRenderModel();

	void showDebugView();

	virtual void setColor(osg::Vec4 color);

private:

	boost::shared_ptr<Mesh> partA_;

};

}



#endif /* ROBOGEN_COLOR_SENSOR_RENDER_MODEL_H_ */
