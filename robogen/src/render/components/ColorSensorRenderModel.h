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
/**
 * \brief Class to render ColorSensor model
 */
class ColorSensorRenderModel: public RenderModel {

public:
	/**
	 * \brief Constuctor
	 */
	ColorSensorRenderModel(boost::shared_ptr<ColorSensorModel> model);
	/**
	 * \brief Destructor
	 */
	virtual ~ColorSensorRenderModel();
	/**
	 * \brief Initializes render model
	 */
	virtual bool initRenderModel();
	/**
	 * \brief Switches to debug view
	 */
	void showDebugView();
	/**
	 * \brief Sets model color
	 */
	virtual void setColor(osg::Vec4 color);

private:

	boost::shared_ptr<Mesh> partA_;

};

}



#endif /* ROBOGEN_COLOR_SENSOR_RENDER_MODEL_H_ */
