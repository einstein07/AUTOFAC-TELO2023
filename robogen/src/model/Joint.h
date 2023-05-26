/*
 * @(#) Joint.h   1.0   September 16, 2015
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Joshua Auerbach
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

#ifndef ROBOGEN_JOINT_H_
#define ROBOGEN_JOINT_H_

#include "Robogen.h"

#include <boost/enable_shared_from_this.hpp>
#include <map>

namespace robogen {

class SimpleBody;
/**
 * \brief Class to describe a Joint
 */
class Joint : public boost::enable_shared_from_this<Joint> {
public:
	/**
	 * \brief Joint type
	 */
	enum JointType {
		FIXED = 0,
		HINGE,
		UNIVERSAL
	};

	/**
	 * \brief Error free constructor
	 *
	 */
	inline Joint() {
		joint_ = NULL;
		fback_ = NULL;
	}
	/**
	 * \brief Creates a hinge
	 *
	 */
	void createHinge(dWorldID world, boost::shared_ptr<SimpleBody> bodyA,
		  boost::shared_ptr<SimpleBody> bodyB,
		  osg::Vec3 axis, osg::Vec3 anchor, dJointGroupID jointGroup=0);
	/**
	 * \brief Creates a fixed Joint
	 */
	void createFixed(dWorldID world, boost::shared_ptr<SimpleBody> bodyA,
		  boost::shared_ptr<SimpleBody> bodyB, dJointGroupID jointGroup=0);
	/**
	 * Creates universal Joint
	 */
	void createUniversal(dWorldID world, boost::shared_ptr<SimpleBody> bodyA,
		  boost::shared_ptr<SimpleBody> bodyB, osg::Vec3 axis1,
		  osg::Vec3 axis2, osg::Vec3 anchor, dJointGroupID jointGroup=0);
	/**
	 * \brief Reconnects Joint connection
	 */
	void reconnect();
	/**
	 * \brief Resets Joint connection
	 */
	void reset();
	/**
	 * \brief Destructor
	 */
	~Joint();
	/**
	 * \brief Returns Joint
	 */
	inline dJointID getJoint() { return joint_; }
	/**
	 * \brief Returns type
	 */
	inline JointType getType() { return type_; }
	/**
	 * \brief Returns body A
	 */
	inline boost::weak_ptr<SimpleBody> getBodyA() { return bodyA_; }
	/**
	 * \brief Returns body B
	 */
	inline boost::weak_ptr<SimpleBody> getBodyB() { return bodyB_; }
	/**
	 * \brief Returns anchor
	 */
	inline const osg::Vec3 &getAnchor() { return anchor_; }
	/**
	 * \brief Returns hinge axis
	 */
	inline const osg::Vec3 &getHingeAxis() { return hingeAxis_; }
	/**
	 * \brief Returns universal axis a
	 */
	inline const osg::Vec3 &getUniversalAxis1() { return universalAxis1_; }
	/**
	 * \brief Returns universal axis 2
	 */
	inline const osg::Vec3 &getUniversalAxis2() { return universalAxis2_; }
	/**
	 * \brief UPdates axis and angle
	 */
	void updateAxisAndAngle();
	/**
	 * \brief Returns world
	 */
	inline dWorldID getWorld() { return world_; }
	/**
	 * \brief Sets parameters
	 */
	void setParam(unsigned int param, double value);
	/**
	 * \brief Sets feedback
	 */
	void setFeedback(dJointFeedback *fback);

private :
	void updateJointParams();


	dJointID joint_;
	boost::weak_ptr<SimpleBody> bodyA_, bodyB_;
	dJointGroupID jointGroup_;
	JointType type_;
	osg::Vec3 anchor_, hingeAxis_, universalAxis1_, universalAxis2_;
	dWorldID world_;
	std::map<unsigned int, double> params_;
	dJointFeedback *fback_;

};


}




#endif /* ROBOGEN_JOINT_H_ */
