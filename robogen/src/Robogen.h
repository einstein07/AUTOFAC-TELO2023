/*
 * @(#) Robogen.h   1.0   Feb 8, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
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
#ifndef ROBOGEN_H_
#define ROBOGEN_H_

// In case of inertia/mass problems with ODE on 64bits machines it's necessary
// to enble dDOUBLE
#define dDOUBLE
#include <limits>
#include <ode/ode.h>
#include <osg/Vec3>
#include <osg/Quat>

#define inMm(x) ((x)/1000.0)
#define inGrams(x) ((x)/1000.0)

namespace robogen {
//---------------------------------------------
/** \brief Useful data for sensing mechanism - SM*/
//---------------------------------------------
struct ObjectData {
	/**
	 * \brief id
	 */
	int objectId;
	/**
	 * \brief Flag robot
	 */
	bool isRobot;
	/**
	 * \brief Flag resource
	 */
	bool isResource;
	/**
	 * \brief Flag target area
	 */
	bool isTargetArea;
	/**
	 * \brief Flag wall
	 */
	bool isWall;
};
//---------------------------------------------
/**
 * \brief ODE ops
 */
float fromOde(float x);
/**
 * \brief ODE ops
 */
double fromOde(double x);
/**
 * \brief ODE ops
 */
osg::Vec3 fromOde(osg::Vec3 x);

/** \brief build rotationMatrixOde and put it in rotationMatrixOde*/
void getRotationMatrixOde(osg::Quat quat, dQuaternion rotationMatrixOde);
/**
 * \brief Returns RPY from Quat
 */
osg::Vec3 getRPYfromQuat(osg::Quat quat);
/**
 * \brief Perform modulo
 */
int modulo(int x, int y);
/**
 * \brief Start robogen
 */
void startRobogen();
/**
 * \brief Exit robogen
 */
void exitRobogen(int exitCode);

}

#endif /* ROBOGEN_H_ */
