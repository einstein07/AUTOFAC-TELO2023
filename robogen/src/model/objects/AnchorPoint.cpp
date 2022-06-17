/*
 * AnchorPoint.cpp
 *
 *  Created on: 27 Oct 2021
 *      Author: root07
 */




/*
 * @(#) Heuristic.h   1.0   Oct 27, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#include "AnchorPoint.h"
namespace robogen{

		AnchorPoint::AnchorPoint(dBodyID bodyId, osg::Vec3d localPosition,
                /**BoxResource::*/Face face): box_(bodyId), localPosition_(localPosition),
                    face_(face), taken_(false){
        }
        AnchorPoint::AnchorPoint(){}

        void AnchorPoint::markTaken() {
            if (!taken_)
                taken_ = true;
            else
                std::cerr<<"Attachment point is already taken!"<<std::endl;

        }

        AnchorPoint::~AnchorPoint(){}



        osg::Vec3d AnchorPoint::getPosition() {
            dVector3 result;
            dBodyGetRelPointPos (
                            box_,
                            localPosition_.x(),
                            localPosition_.y(),
                            localPosition_.z(),
                            result
                        );

            return osg::Vec3d(result[0], result[1], result[2]);
        }



}

