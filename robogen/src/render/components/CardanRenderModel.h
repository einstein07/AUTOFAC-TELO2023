/*
 * @(#) CardanRenderModel.h   1.0   Feb 13, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#ifndef ROBOGEN_CARDAN_RENDER_MODEL_H_
#define ROBOGEN_CARDAN_RENDER_MODEL_H_

#include "model/components/CardanModel.h"
#include "render/RenderModel.h"

namespace robogen {

class Mesh;
/**
 * Class to render Cardan model
 */
class CardanRenderModel : public RenderModel {

public:
	/**
	 * \brief Constructor
	 */
	CardanRenderModel(boost::shared_ptr<CardanModel> model);
   /**
    * \brief Destructor
    */
   virtual ~CardanRenderModel();
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
   boost::shared_ptr<Mesh> partB_;
   boost::shared_ptr<Mesh> patCross_;
   osg::ref_ptr<osg::PositionAttitudeTransform> jointPat_;

};

}


#endif /* ROBOGEN_CARDAN_RENDER_MODEL_H_ */
