/*
 * @(#) BoxResourceRender.h   1.0   August 02, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#ifndef ROBOGEN_BOX_RESOURCE_RENDER_MODEL_H_
#define ROBOGEN_BOX_RESOURCE_RENDER_MODEL_H_

#include <boost/shared_ptr.hpp>

#include <osg/Node>
#include <osg/PositionAttitudeTransform>

namespace robogen {

class BoxResource;

class BoxResourceRender {

public:

	BoxResourceRender(boost::shared_ptr<BoxResource> resource);

	virtual ~BoxResourceRender();

	osg::ref_ptr<osg::PositionAttitudeTransform> getRootNode();



private:

	/**
	 * The root Position Attitude Transform defines the relative position of
	 * the component with respect to the parent component
	 */
	osg::ref_ptr<osg::PositionAttitudeTransform> rootNode_;

};


}


#endif /* ROBOGEN_BOX_RESOURCE_RENDER_MODEL_H_ */
