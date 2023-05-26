/*
 * @(#) TargetAreaDetectorRenderModel.h   1.0   Oct 16, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef ROBOGEN_TARGET_AREA_DETECTOR_RENDER_MODEL_H_
#define ROBOGEN_TARGET_AREA_DETECTOR_RENDER_MODEL_H_

#include "model/components/perceptive/TargetAreaDetectorModel.h"
#include "render/RenderModel.h"

namespace robogen {

class Mesh;
/**
 * \brief Class to render TargetAreaDetector model
 */
class TargetAreaDetectorRenderModel: public RenderModel {

public:
	/**
	 * \brief Constructor
	 */
	TargetAreaDetectorRenderModel(boost::shared_ptr<TargetAreaDetectorModel> model);
	/**
	 * \brief Destructor
	 */
	virtual ~TargetAreaDetectorRenderModel();
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



#endif /* ROBOGEN_TARGET_AREA_DETECTOR_RENDER_MODEL_H_ */
