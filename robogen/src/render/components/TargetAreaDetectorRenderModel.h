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

class TargetAreaDetectorRenderModel: public RenderModel {

public:

	TargetAreaDetectorRenderModel(boost::shared_ptr<TargetAreaDetectorModel> model);

	virtual ~TargetAreaDetectorRenderModel();

	virtual bool initRenderModel();

	void showDebugView();

	virtual void setColor(osg::Vec4 color);

private:

	boost::shared_ptr<Mesh> partA_;

};

}



#endif /* ROBOGEN_TARGET_AREA_DETECTOR_RENDER_MODEL_H_ */
