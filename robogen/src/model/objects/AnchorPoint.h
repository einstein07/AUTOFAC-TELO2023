/*
 * @(#) Heuristic.h   1.0   Oct 27, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef EDQD_ANCHORPOINT_H_
#define EDQD_ANCHORPOINT_H_

#include <osg/Quat>
#include <osg/Vec3>
#include <cmath>
#include "Obstacle.h"
#include "Robogen.h"
#include "Robot.h"
//#include "BoxResource.h"
namespace robogen{
	enum Face {
	            NONE, LEFT, RIGHT, BACK, FRONT
	        };
class AnchorPoint{

    public:


        /**
         * Initializes an anchor point
         * @param position position local to the resource
         */
        AnchorPoint(dBodyID bodyId, osg::Vec3 localPosition,
                /**BoxResource::*/Face face);
        AnchorPoint();
        /**
         * Mark this point as taken
         */
        void markTaken();

        /**
         * Destructor
         */
        ~AnchorPoint();

        /**
         * @return the local position
         */
        const osg::Vec3d getLocalPosition() {
            return localPosition_;
        }

        /**
         * @return the global position
         */
        osg::Vec3d getPosition();

        /**
         * @return the box face
         */
        const /**BoxResource::*/Face getFace() {
            return face_;
        }

        /**
         * @return whether this point is taken - true - or not - false.
         */
        const bool isTaken() {
            return taken_;
        }

    private:

        /**
        * The box
        */
       dBodyID box_;
        /**
         * Local position
         */
        osg::Vec3 localPosition_;

        /**
         * Status of the anchor point
         */
        bool taken_;

        /**
         * Which face of the cube
         */
        /**BoxResource::*/Face face_;

};

}

#endif /* EDQD_ANCHORPOINT_H_ */
